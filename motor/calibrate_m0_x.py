#!/usr/bin/env python3
"""Calibrate an AS5048A SPI encoder on M0 and save its calibration."""
from .calibrate_m0 import cli


if __name__ == "__main__":
    cli(absolute=True)
