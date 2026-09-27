# -*- coding: utf-8 -*-
"""
Class Work 8: Main Launcher.
Redirects to code/slam.py for modular execution.

Usage:
    python run.py --sim
    python run.py
    python run.py --sim --no-dashboard --start-x 1 --start-y 1 --start-heading NORTH
"""

import os
import sys

# Ensure the 'code' directory is on Python search path
CODE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "code")
if CODE_DIR not in sys.path:
    sys.path.insert(0, CODE_DIR)

# pyrefly: ignore [missing-import]
from slam import main
# pyrefly: ignore [missing-import]
from evaluation import re_evaluate_from_saved

if __name__ == "__main__":
    if "--eval" in sys.argv:
        re_evaluate_from_saved()
    else:
        main()

