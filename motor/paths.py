"""Paths shared by desktop and terminal workflows."""
from pathlib import Path

WORKSPACE = Path(__file__).resolve().parents[1]
CONFIG_PATH = WORKSPACE / "motor_config.json"
LOGS = WORKSPACE / "logs"
MOTION_LOCK = LOGS / ".motion.lock"
