# 🧩 `scr/mods/`

> Custom modification scripts and shared utilities for the Isaac Lab project environment.  
> Designed to consolidate reusable logic, streamline workflows, and extend base functionality without polluting the original source.

---

## 📦 Contents

This folder is intended to house **custom-built modules** that support scripting and configuration across your Isaac Lab workflows. These are **not part of upstream Isaac Lab**, but are used alongside or in place of upstream scripts.

### Current Modules

```bash
scr/mods/
├── app_parser.py      # Centralized argument parser shared across scripts
├── utils.py           # Logging, callbacks, and shared helper functions
```

### Potential Future Additions

- `teleop_helpers.py` — abstraction layer over keyboard/gamepad input  
- `logging_ext.py` — custom formatters and log filters  
- `recorder_wrapper.py` — extensions for HDF5/video recorders  
- `scene_loader.py` — common scene/environment loading logic  

---

## 🧠 Purpose

This module directory exists to:

- Consolidate repeated logic from demo scripts  
- Serve as a base layer for your custom experiments or modified training loops  
- Ensure clear separation between core framework and project-specific tools  
- Provide a safe location to iterate without risking damage to reference code  

---

## 🛠️ Usage

You can import these modules from any script inside your project by modifying the `PYTHONPATH` or appending relative paths in scripts:

```python
from scr.mods.app_parser import build_arg_parser
from scr.mods.utils import get_logger, register_common_callbacks
```

Ensure the root directory is in your Python path, e.g.:

```bash
export PYTHONPATH=$(pwd):$PYTHONPATH
```

Or update `sys.path` at runtime (not ideal but quick):

```python
import sys
sys.path.append("path/to/root")
```

---

## ⚠️ Note

This folder is **actively developed** and **project-specific**. If any of the utilities here become general-purpose or widely reused, consider promoting them to a proper `src/` package or `pip` module for better maintainability.

---

## ✅ Conventions

- Scripts should follow [PEP8](https://peps.python.org/pep-0008/) + [Google style guide](https://google.github.io/styleguide/pyguide.html)  
- Avoid hardcoding paths — make use of `argparse` or config loaders  
- Include docstrings and function-level comments  
- Favor composability: keep functions focused and importable  

---

## 📌 Related Folders

- [`reference_scripts/`](../reference_scripts/) — Unmodified upstream Isaac Lab scripts  
- `scr/tasks/` — Task-specific logic  
- `scr/scripts/` — Top-level runnable scripts using these shared mods  
