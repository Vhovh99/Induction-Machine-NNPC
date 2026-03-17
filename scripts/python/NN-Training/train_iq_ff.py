"""
iq_ff Neural Network Training  —  Residual Learning
=====================================================
Predicts the RESIDUAL current that the PI integrator cannot anticipate:

  Δiq = iq - iq_I_term
      ≈ (J / Kt(imr)) * dwr_dt  +  T_friction(omega_m) / Kt(imr)

With the load-torque component (iq_I_term) removed from the target, the
NN only has to model the deterministic mechanical terms, which are tightly
correlated with dwr_dt and omega_m.  The PI integrator continues to handle
the slowly-varying load torque on the MCU.

On the MCU:
  iq_ff = iq_ff_nn(omega_m, omega_m_ref, dwr_dt, imr) + iq_I_term

Inputs : omega_m, omega_m_ref, dwr_dt, imr
Target : iq - iq_I_term  (mechanical feedforward residual)
"""

import csv
import math
import random
import os
import json

import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, TensorDataset
import matplotlib.pyplot as plt
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import mean_absolute_error, mean_squared_error, r2_score

# ── Reproducibility ────────────────────────────────────────────────────────────
SEED = 42
random.seed(SEED)
np.random.seed(SEED)
torch.manual_seed(SEED)

# ── Config ─────────────────────────────────────────────────────────────────────
DATA_FILES  = [
    "nn_training_data1.csv",
    "nn_training_data2.csv",
    "nn_training_data3.csv",
    "nn_training_data4.csv",
]
MODEL_DIR   = "model"
EPOCHS      = 1000
BATCH_SIZE  = 256
LR          = 1e-2         # higher LR needed for tanh to converge in ~1000 epochs
PATIENCE    = 60          # early-stopping patience (epochs without val improvement)
VAL_SPLIT   = 0.15
TEST_SPLIT  = 0.10
HIDDEN      = [16, 8]      # 4 → 16 → 8 → 1  (MCU-friendly, best R² at low param count)

# Noise filter: rows where |dwr_dt| exceeds this threshold are discarded.
# The raw signal is a numerical derivative of omega_m and contains large
# transient spikes (p99 ≈ ±245 rad/s²; extremes reach ±6350 rad/s²).
# 500 rad/s² keeps >99% of the data while removing differentiation artefacts.
# Set to None to disable filtering.
DWR_DT_CLIP = 150.0   # rad/s²

# Physics-motivated inputs (see module docstring)
# omega_m_ref added: gives the NN the speed error context so it can
# distinguish steady-state cruising from active acceleration commands.
INPUT_COLS  = ["omega_m", "omega_m_ref", "dwr_dt", "imr"]
TARGET_COL  = "iq"          # raw column in CSV
RESIDUAL    = True          # if True, train on (iq - iq_I_term) instead of iq


# ── 1. Load data ───────────────────────────────────────────────────────────────
def load_csv(paths):
    """Load and concatenate one or more CSV files."""
    if isinstance(paths, str):
        paths = [paths]

    all_rows = []
    for path in paths:
        with open(path, newline="") as f:
            reader = csv.DictReader(f)
            file_rows = list(reader)
        print(f"  {path}: {len(file_rows)} rows")
        all_rows.extend(file_rows)

    before = len(all_rows)
    if DWR_DT_CLIP is not None:
        all_rows = [r for r in all_rows if abs(float(r["dwr_dt"])) <= DWR_DT_CLIP]
        dropped = before - len(all_rows)
        print(f"  dwr_dt filter |dwr_dt| ≤ {DWR_DT_CLIP}: "
              f"kept {len(all_rows)}/{before} rows, dropped {dropped} ({100*dropped/before:.1f}%)")

    X = np.array([[float(r[c]) for c in INPUT_COLS] for r in all_rows], dtype=np.float32)
    if RESIDUAL:
        y = np.array([float(r["iq"]) - float(r["iq_I_term"]) for r in all_rows],
                     dtype=np.float32).reshape(-1, 1)
    else:
        y = np.array([float(r[TARGET_COL]) for r in all_rows], dtype=np.float32).reshape(-1, 1)
    return X, y


# ── 2. Split ───────────────────────────────────────────────────────────────────
def split(X, y, val_frac, test_frac, seed=SEED):
    n = len(X)
    idx = np.random.default_rng(seed).permutation(n)
    n_test = int(n * test_frac)
    n_val  = int(n * val_frac)
    test_idx = idx[:n_test]
    val_idx  = idx[n_test:n_test + n_val]
    train_idx = idx[n_test + n_val:]
    return (X[train_idx], y[train_idx],
            X[val_idx],   y[val_idx],
            X[test_idx],  y[test_idx])


# ── 3. Model ───────────────────────────────────────────────────────────────────
class IqFFNet(nn.Module):
    """
    Compact MLP with Tanh activations: 4 → 8 → 1.
    Tanh is bounded [-1, 1], numerically safe on fixed-point MCUs, and
    matches the smooth, monotone physics of the mechanical equation.
    """
    def __init__(self, n_in: int, hidden: list[int]):
        super().__init__()
        layers = []
        prev = n_in
        for h in hidden:
            layers += [nn.Linear(prev, h), nn.Tanh()]
            prev = h
        layers.append(nn.Linear(prev, 1))   # linear output (regression)
        self.net = nn.Sequential(*layers)

    def forward(self, x):
        return self.net(x)


# ── 4. Training loop ───────────────────────────────────────────────────────────
def train(model, optimizer, criterion, loader):
    model.train()
    total_loss = 0.0
    for xb, yb in loader:
        optimizer.zero_grad()
        loss = criterion(model(xb), yb)
        loss.backward()
        optimizer.step()
        total_loss += loss.item() * len(xb)
    return total_loss / len(loader.dataset)


@torch.no_grad()
def evaluate(model, criterion, loader):
    model.eval()
    total_loss = 0.0
    for xb, yb in loader:
        total_loss += criterion(model(xb), yb).item() * len(xb)
    return total_loss / len(loader.dataset)


# ── 5. Main ────────────────────────────────────────────────────────────────────
def main():
    os.makedirs(MODEL_DIR, exist_ok=True)

    # --- Load & split ---
    print(f"Loading {len(DATA_FILES)} file(s) ...")
    X, y = load_csv(DATA_FILES)
    print(f"  Total samples : {len(X)}")
    effective_target = "iq - iq_I_term (residual)" if RESIDUAL else TARGET_COL
    print(f"  Inputs        : {INPUT_COLS}")
    print(f"  Target        : {effective_target}")

    X_train, y_train, X_val, y_val, X_test, y_test = split(X, y, VAL_SPLIT, TEST_SPLIT)
    print(f"  Train/Val/Test: {len(X_train)} / {len(X_val)} / {len(X_test)}")

    # --- Normalise (fit on train only, to prevent data leakage) ---
    scaler_X = StandardScaler().fit(X_train)
    scaler_y = StandardScaler().fit(y_train)

    def scale(Xr, yr):
        return (torch.tensor(scaler_X.transform(Xr)),
                torch.tensor(scaler_y.transform(yr)))

    Xt, yt     = scale(X_train, y_train)
    Xv, yv     = scale(X_val,   y_val)
    Xte, yte   = scale(X_test,  y_test)

    train_loader = DataLoader(TensorDataset(Xt, yt),   batch_size=BATCH_SIZE, shuffle=True)
    val_loader   = DataLoader(TensorDataset(Xv, yv),   batch_size=BATCH_SIZE)
    test_loader  = DataLoader(TensorDataset(Xte, yte), batch_size=BATCH_SIZE)

    # --- Build model ---
    model     = IqFFNet(n_in=len(INPUT_COLS), hidden=HIDDEN)
    optimizer = torch.optim.Adam(model.parameters(), lr=LR)
    scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
        optimizer, factor=0.5, patience=10, min_lr=1e-6)
    criterion = nn.MSELoss()

    print(f"\nModel: {model}")
    total_params = sum(p.numel() for p in model.parameters())
    print(f"Parameters: {total_params}\n")

    # --- Training ---
    best_val  = math.inf
    patience_counter = 0
    train_losses, val_losses = [], []

    for epoch in range(1, EPOCHS + 1):
        tr_loss  = train(model, optimizer, criterion, train_loader)
        val_loss = evaluate(model, criterion, val_loader)
        scheduler.step(val_loss)

        train_losses.append(tr_loss)
        val_losses.append(val_loss)

        if val_loss < best_val:
            best_val = val_loss
            torch.save(model.state_dict(), os.path.join(MODEL_DIR, "iq_ff_best.pt"))
            patience_counter = 0
        else:
            patience_counter += 1

        if epoch % 20 == 0 or epoch == 1:
            print(f"Epoch {epoch:4d} | train MSE={tr_loss:.6f}  val MSE={val_loss:.6f}"
                  f"  lr={optimizer.param_groups[0]['lr']:.2e}")

        if patience_counter >= PATIENCE:
            print(f"\nEarly stopping at epoch {epoch} (no val improvement for {PATIENCE} epochs)")
            break

    # --- Final evaluation on test set ---
    model.load_state_dict(torch.load(os.path.join(MODEL_DIR, "iq_ff_best.pt"), weights_only=True))
    model.eval()

    with torch.no_grad():
        y_pred_scaled = model(Xte).numpy()

    y_pred = scaler_y.inverse_transform(y_pred_scaled)
    y_true = y_test   # original scale

    mae  = mean_absolute_error(y_true, y_pred)
    rmse = math.sqrt(mean_squared_error(y_true, y_pred))
    r2   = r2_score(y_true, y_pred)

    print(f"\n── Test-set metrics (original scale) ──")
    print(f"  MAE  : {mae:.6f} A")
    print(f"  RMSE : {rmse:.6f} A")
    print(f"  R²   : {r2:.6f}")

    # --- Save scaler params and model meta (for embedded / C export) ---
    meta = {
        "input_cols"   : INPUT_COLS,
        "target_col"   : "iq_residual" if RESIDUAL else TARGET_COL,
        "residual_mode": RESIDUAL,
        "hidden_layers": HIDDEN,
        "scaler_X_mean": scaler_X.mean_.tolist(),
        "scaler_X_std" : scaler_X.scale_.tolist(),
        "scaler_y_mean": scaler_y.mean_.tolist(),
        "scaler_y_std" : scaler_y.scale_.tolist(),
        "test_mae_A"   : mae,
        "test_rmse_A"  : rmse,
        "test_r2"      : r2,
    }
    meta_path = os.path.join(MODEL_DIR, "iq_ff_meta.json")
    with open(meta_path, "w") as f:
        json.dump(meta, f, indent=2)
    print(f"\nScaler params + meta saved to {meta_path}")

    # Export full model weights as plain JSON (easy to port to C/embedded)
    weights = {}
    for name, param in model.named_parameters():
        weights[name] = param.detach().numpy().tolist()
    weights_path = os.path.join(MODEL_DIR, "iq_ff_weights.json")
    with open(weights_path, "w") as f:
        json.dump(weights, f, indent=2)
    print(f"Layer weights saved to {weights_path}")

    # --- Plots ---
    fig, axes = plt.subplots(1, 2, figsize=(12, 4))

    # Learning curves
    axes[0].plot(train_losses, label="Train MSE (scaled)")
    axes[0].plot(val_losses,   label="Val MSE (scaled)")
    axes[0].set_xlabel("Epoch")
    axes[0].set_ylabel("MSE (normalised units)")
    axes[0].set_title("Learning Curves")
    axes[0].legend()
    axes[0].grid(True)
    axes[0].set_yscale("log")

    # Parity plot
    axes[1].scatter(y_true, y_pred, alpha=0.3, s=10)
    lim = [min(y_true.min(), y_pred.min()) - 0.05,
           max(y_true.max(), y_pred.max()) + 0.05]
    axes[1].plot(lim, lim, "r--", linewidth=1, label="Perfect")
    xlabel = "True Δiq = iq−iq_I_term (A)" if RESIDUAL else "True iq (A)"
    ylabel = "Predicted Δiq (A)" if RESIDUAL else "Predicted iq_ff (A)"
    axes[1].set_xlabel(xlabel)
    axes[1].set_ylabel(ylabel)
    axes[1].set_title(f"Parity Plot  R²={r2:.4f}")
    axes[1].legend()
    axes[1].grid(True)

    plt.tight_layout()
    plot_path = os.path.join(MODEL_DIR, "training_results.png")
    plt.savefig(plot_path, dpi=150)
    print(f"Plots saved to {plot_path}")
    plt.show()


if __name__ == "__main__":
    main()
