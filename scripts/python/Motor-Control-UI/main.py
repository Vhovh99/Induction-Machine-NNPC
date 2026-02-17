#!/usr/bin/env python3
"""
Motor Control UI - Induction Machine FOC Control Application

A comprehensive Python UI for controlling an induction machine with 
Field-Oriented Control (FOC). Provides real-time monitoring of speed,
torque, and phase currents through serial communication.

Usage:
    python main.py
"""

import sys

# CRITICAL: Set matplotlib backend BEFORE importing PyQt5 or any Qt-dependent modules
import matplotlib
matplotlib.use('Qt5Agg')

from PyQt5.QtWidgets import QApplication
from app_window import MotorControlUI


def main():
    """Main entry point for the application."""
    app = QApplication(sys.argv)
    
    # Create and show the main window
    window = MotorControlUI()
    window.show()
    
    # Start the application
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
