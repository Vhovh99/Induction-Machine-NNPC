# Motor Control UI - Segmentation Fault Fix

## Issue: Segmentation Fault on Startup

### Root Causes
The segmentation fault was caused by two issues:

1. **Matplotlib Backend Initialization Timing**: Matplotlib tried to initialize its Qt5 backend before `QApplication` was created, causing a crash
2. **Python 3.13 Compatibility**: Older numpy and setuptools versions had compatibility issues with Python 3.13

### Solutions Applied

#### Fix #1: Lazy Matplotlib Import
Modified `app_window.py` and `main.py` to defer matplotlib backend initialization until `QApplication` is created:

```python
# In main.py: Set backend BEFORE importing PyQt5
import matplotlib
matplotlib.use('Qt5Agg')

# In app_window.py: Use lazy imports
FigureCanvas = None
Figure = None

def _init_matplotlib():
    """Initialize matplotlib after QApplication is created"""
    global FigureCanvas, Figure
    # ... import and configure here
```

#### Fix #2: Updated Package Versions
Updated to Python 3.13-compatible versions:
- PyQt5: 5.15.11 (was 5.15.9)
- matplotlib: 3.10.8 (was 3.8.2)
- numpy: 2.4.2 (was 1.26.3)

### How to Run

#### Option 1: Command Line (Direct)
```bash
cd Motor-Control-UI
source venv/bin/activate
python main.py
```

#### Option 2: X11 Explicit (if Wayland issues occur)
```bash
source venv/bin/activate
QT_QPA_PLATFORM=xcb python main.py
```

#### Option 3: Create an Alias
```bash
# Add to ~/.bashrc
alias motor-ui='cd ~/Projects/Polytech/Induction-Machine-NNPC/scripts/python/Motor-Control-UI && source venv/bin/activate && python main.py'

# Then just run:
motor-ui
```

#### Option 4: Create Desktop Launcher
Create `~/.local/share/applications/motor-control-ui.desktop`:
```ini
[Desktop Entry]
Type=Application
Name=Motor Control UI
Comment=Induction Machine FOC Control
Exec=bash -c 'cd ~/Projects/Polytech/Induction-Machine-NNPC/scripts/python/Motor-Control-UI && source venv/bin/activate && python main.py'
Terminal=false
Categories=Engineering;
Icon=applications-engineering
```

### If Issues Persist

#### 1. Verify Installation
```bash
python -c "from app_window import MotorControlUI; print('✓ OK')"
```

#### 2. Check Package Versions
```bash
pip list | grep -E "PyQt5|matplotlib|numpy"
```

#### 3. Reinstall in Clean Virtual Environment
```bash
cd Motor-Control-UI
rm -rf venv
python3 -m venv venv
source venv/bin/activate
pip install --only-binary :all: -r requirements.txt
python main.py
```

#### 4. Debug Information
If still getting errors, run with verbose output:
```bash
python -v main.py 2>&1 | head -100
```

### System Requirements Verified
- ✅ Python 3.13
- ✅ PyQt5 5.15.11
- ✅ matplotlib 3.10.8
- ✅ numpy 2.4.2
- ✅ Wayland/X11 display server

### Technical Details

**The Segfault Sequence (Before Fix):**
1. `main.py` imports `app_window` module
2. Module-level matplotlib imports execute
3. Qt5 backend tries to initialize without QApplication
4. → **Segmentation fault**

**Fixed Sequence (After Fix):**
1. `main.py` creates QApplication first
2. `main.py` imports `app_window` module  
3. Module-level matplotlib imports are deferred (lazy)
4. `app_window.__init__()` calls `_init_matplotlib()`
5. Matplotlib initializes safely with QApplication active
6. → **Application runs successfully**

### Files Modified
- `main.py` - Added matplotlib backend configuration and QApplication before imports
- `app_window.py` - Implemented lazy matplotlib imports with `_init_matplotlib()` function
- `requirements.txt` - Updated package versions for Python 3.13 compatibility

### Version History
- **v1.0** (2026-02-17): Released with original versions (segfault issues)
- **v1.1** (2026-02-17): Fixed - Lazy imports and updated dependencies

### Notes for Development
- Always create `QApplication` before importing UI modules
- Use lazy imports for display-dependent libraries when at module level
- Test on Python 3.13 before release (compatibility improved significantly)
- Binary wheels (--only-binary :all:) are recommended for stability
