"""
Model Conversion for STM32Cube.AI
===================================
Produces three deployment artefacts from the trained iq_ff model:

  model/iq_ff.onnx          – for STM32Cube.AI (X-CUBE-AI) direct import
  model/iq_ff_keras.keras    – Keras SavedModel (also accepted by Cube.AI)
  model/iq_ff_float32.tflite – TFLite float32 (TF Lite for Microcontrollers)
  model/iq_ff_int8.tflite    – TFLite INT8 quantised  (smallest, fastest on CM4/CM7)

STM32Cube.AI import order of preference:
  1. ONNX  › File → "Analyse" → choose iq_ff.onnx
  2. Keras › File → "Analyse" → choose iq_ff_keras.keras
  3. TFLite › iq_ff_int8.tflite  (use if targeting very limited flash/RAM)

The Keras model is rebuilt directly from the saved weight JSON so it is
completely independent of PyTorch and avoids any onnx-conversion chains.
"""

import json
import os
import sys

import numpy as np
import torch
import torch.nn as nn

MODEL_DIR  = "model"
META_FILE  = os.path.join(MODEL_DIR, "iq_ff_meta.json")
PT_FILE    = os.path.join(MODEL_DIR, "iq_ff_best.pt")
WEIGHTS_FILE = os.path.join(MODEL_DIR, "iq_ff_weights.json")

# ── Load meta ──────────────────────────────────────────────────────────────────
with open(META_FILE) as f:
    meta = json.load(f)

HIDDEN     = meta["hidden_layers"]   # e.g. [64, 64, 32]
N_IN       = len(meta["input_cols"]) # 3
INPUT_COLS = meta["input_cols"]


# ── Shared model definition (must match train_iq_ff.py) ───────────────────────
class IqFFNet(nn.Module):
    def __init__(self, n_in: int, hidden: list):
        super().__init__()
        layers = []
        prev = n_in
        for h in hidden:
            layers += [nn.Linear(prev, h), nn.ReLU()]
            prev = h
        layers.append(nn.Linear(prev, 1))
        self.net = nn.Sequential(*layers)

    def forward(self, x):
        return self.net(x)


# ═══════════════════════════════════════════════════════════════════════════════
# 1.  ONNX export (PyTorch native, most reliable)
# ═══════════════════════════════════════════════════════════════════════════════
def export_onnx():
    print("\n── Step 1: PyTorch → ONNX ──")
    model = IqFFNet(N_IN, HIDDEN)
    model.load_state_dict(torch.load(PT_FILE, weights_only=True))
    model.eval()

    dummy = torch.zeros(1, N_IN)   # batch=1, 3 inputs
    out_path = os.path.join(MODEL_DIR, "iq_ff.onnx")

    torch.onnx.export(
        model,
        dummy,
        out_path,
        input_names=["input"],        # name the input tensor
        output_names=["iq_ff"],       # name the output tensor
        dynamic_axes={"input": {0: "batch"}, "iq_ff": {0: "batch"}},
        opset_version=18,             # STM32Cube.AI supports up to opset 18
    )

    # Verify the graph is well-formed
    import onnx
    onnx.checker.check_model(out_path)
    print(f"  Saved + verified: {out_path}")

    # Print input/output metadata for reference
    model_onnx = onnx.load(out_path)
    for inp in model_onnx.graph.input:
        print(f"  ONNX input  : {inp.name}  shape={[d.dim_value for d in inp.type.tensor_type.shape.dim]}")
    for out in model_onnx.graph.output:
        print(f"  ONNX output : {out.name}  shape={[d.dim_value for d in out.type.tensor_type.shape.dim]}")


# ═══════════════════════════════════════════════════════════════════════════════
# 2.  Keras model rebuilt from saved weights  →  .keras  +  TFLite
# ═══════════════════════════════════════════════════════════════════════════════
def export_keras_tflite():
    print("\n── Step 2: Weights JSON → Keras ──")
    os.environ["TF_CPP_MIN_LOG_LEVEL"] = "3"   # suppress TF spam
    import tensorflow as tf

    # --- Build equivalent Keras model ---
    inputs = tf.keras.Input(shape=(N_IN,), name="input")
    x = inputs
    layer_idx = 0
    for h in HIDDEN:
        x = tf.keras.layers.Dense(h, activation="relu", name=f"dense_{layer_idx}")(x)
        layer_idx += 1
    outputs = tf.keras.layers.Dense(1, activation="linear", name="iq_ff")(x)
    keras_model = tf.keras.Model(inputs, outputs)

    # --- Copy weights from saved JSON ---
    with open(WEIGHTS_FILE) as f:
        saved_w = json.load(f)

    # Map PyTorch sequential layer indices to Keras dense layers.
    # PyTorch net: (0)Linear (1)ELU (2)Linear (3)ELU ... (N)Linear
    # keras_model.layers[0] is the Input layer; Dense layers follow.
    # We keep a separate counter pt_idx so the Input layer doesn't offset us.
    pt_idx = 0   # increments by 2 (Linear stride) per Dense layer found
    for layer in keras_model.layers:
        if not isinstance(layer, tf.keras.layers.Dense):
            continue
        pt_key_w = f"net.{pt_idx}.weight"
        pt_key_b = f"net.{pt_idx}.bias"
        pt_idx += 2
        # PyTorch Linear stores weight as (out, in); Keras expects (in, out)
        W = np.array(saved_w[pt_key_w], dtype=np.float32).T
        b = np.array(saved_w[pt_key_b], dtype=np.float32)
        layer.set_weights([W, b])
        print(f"  Loaded layer '{layer.name}': W={W.shape}  b={b.shape}")

    keras_path = os.path.join(MODEL_DIR, "iq_ff_keras.keras")
    keras_model.save(keras_path)
    print(f"  Saved: {keras_path}")

    # --- Quick numerical sanity check vs PyTorch ---
    torch_model = IqFFNet(N_IN, HIDDEN)
    torch_model.load_state_dict(torch.load(PT_FILE, weights_only=True))
    torch_model.eval()

    rng = np.random.default_rng(0)
    test_X = rng.standard_normal((10, N_IN)).astype(np.float32)
    with torch.no_grad():
        pt_out = torch_model(torch.tensor(test_X)).numpy().flatten()
    kx_out = keras_model.predict(test_X, verbose=0).flatten()
    max_diff = np.abs(pt_out - kx_out).max()
    print(f"  Max numerical diff PyTorch vs Keras: {max_diff:.2e}"
          f"  {'✓ OK' if max_diff < 1e-5 else '✗  WARNING — weight mismatch!'}")

    # ── TFLite float32 ──────────────────────────────────────────────────────
    print("\n── Step 3: Keras → TFLite float32 ──")
    converter = tf.lite.TFLiteConverter.from_keras_model(keras_model)
    tflite_f32 = converter.convert()
    f32_path = os.path.join(MODEL_DIR, "iq_ff_float32.tflite")
    with open(f32_path, "wb") as f:
        f.write(tflite_f32)
    print(f"  Saved: {f32_path}  ({len(tflite_f32)/1024:.1f} kB)")

    # ── TFLite INT8 full-integer quantisation ───────────────────────────────
    print("\n── Step 4: Keras → TFLite INT8 (full-integer) ──")
    # Representative dataset: use normalised inputs (same scaler as training)
    scaler_mean = np.array(meta["scaler_X_center"], dtype=np.float32)
    scaler_std  = np.array(meta["scaler_X_scale"],  dtype=np.float32)

    def representative_dataset():
        # The Keras/TFLite model always receives NORMALISED inputs.
        # Simulate the RobustScaler output: approximately N(0,1) in bulk,
        # with heavier tails (especially for dwr_dt).
        rng2 = np.random.default_rng(1)
        for _ in range(500):
            z = rng2.standard_normal((1, N_IN)).astype(np.float32)
            yield [z]

    converter_int8 = tf.lite.TFLiteConverter.from_keras_model(keras_model)
    converter_int8.optimizations = [tf.lite.Optimize.DEFAULT]
    converter_int8.representative_dataset = representative_dataset
    converter_int8.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
    converter_int8.inference_input_type  = tf.int8
    converter_int8.inference_output_type = tf.int8

    tflite_int8 = converter_int8.convert()
    int8_path = os.path.join(MODEL_DIR, "iq_ff_int8.tflite")
    with open(int8_path, "wb") as f:
        f.write(tflite_int8)
    print(f"  Saved: {int8_path}  ({len(tflite_int8)/1024:.1f} kB)")

    # ── Verify INT8 model output (qualitative) ───────────────────────────────
    interp = tf.lite.Interpreter(model_content=tflite_int8)
    interp.allocate_tensors()
    inp_det  = interp.get_input_details()[0]
    out_det  = interp.get_output_details()[0]

    in_scale, in_zero   = inp_det["quantization"]
    out_scale, out_zero = out_det["quantization"]

    # test_X is already in normalised space (standard-normal samples)
    errors = []
    for i in range(len(test_X)):
        xi_q = np.round(test_X[i] / in_scale + in_zero).astype(np.int8).reshape(1, N_IN)
        interp.set_tensor(inp_det["index"], xi_q)
        interp.invoke()
        y_q = interp.get_tensor(out_det["index"])[0, 0]
        y_f = (float(y_q) - out_zero) * out_scale   # normalised-space float
        errors.append(abs(kx_out[i] - y_f))

    print(f"  INT8 vs Keras max diff (normalised space): {max(errors):.4f}")

    # ── Print quantisation params for C code ─────────────────────────────────
    print("\n── Quantisation params (needed in C inference code) ──")
    print(f"  Input  scale={in_scale:.6f}  zero_point={in_zero}")
    print(f"  Output scale={out_scale:.6f}  zero_point={out_zero}")

    scaler_center = meta["scaler_X_center"]
    scaler_scale  = meta["scaler_X_scale"]

    return keras_model


# ═══════════════════════════════════════════════════════════════════════════════
# 3.  Print STM32Cube.AI instructions
# ═══════════════════════════════════════════════════════════════════════════════
def print_stm32_instructions():
    scaler_center = meta["scaler_X_center"]
    scaler_scale  = meta["scaler_X_scale"]
    print("""
══════════════════════════════════════════════════════════════
  STM32Cube.AI  (X-CUBE-AI)  import instructions
══════════════════════════════════════════════════════════════

Option A – ONNX  [recommended, most accurate]
  STM32CubeIDE → X-CUBE-AI → "Add network"
  → Select file: model/iq_ff.onnx
  → Framework: ONNX
  → Analyse / Validate → Generate Code

Option B – Keras
  → Select file: model/iq_ff_keras.keras
  → Framework: Keras

Option C – TFLite INT8  [smallest flash footprint]
  → Select file: model/iq_ff_int8.tflite
  → Framework: TFLite

──────────────────────────────────────────────────────────────
  Pre-processing (RobustScaler — values from model/iq_ff_meta.json):

    center = {center}   (median)
    scale  = {scale}    (IQR)

  For float32 (ONNX / Keras / TFLite-f32):
    input[0] = (omega_m      - center[0]) / scale[0]
    input[1] = (omega_m_ref  - center[1]) / scale[1]
    input[2] = (dwr_dt       - center[2]) / scale[2]
    input[3] = (imr          - center[3]) / scale[3]

  For INT8 (TFLite-int8):
    input[i] = (int8_t)round( (x_norm[i] / in_scale) + in_zero )

  Post-processing (float32):
    iq_ff = output[0] * scaler_y_std + scaler_y_mean

  Post-processing (INT8):
    y_norm = ((float)raw_output - out_zero) * out_scale
    iq_ff  = y_norm * scaler_y_std + scaler_y_mean

  Input column order in the network:  {cols}
══════════════════════════════════════════════════════════════
""".format(
        center=scaler_center,
        scale=scaler_scale,
        cols=meta["input_cols"],
    ))


# ═══════════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    os.makedirs(MODEL_DIR, exist_ok=True)

    export_onnx()
    export_keras_tflite()
    print_stm32_instructions()

    print("Done. All artefacts are in the model/ directory.\n")
