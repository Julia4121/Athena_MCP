"""Runtime constants shared by the simulation tools."""

from pathlib import Path

PLUGIN_ROOT = Path(__file__).resolve().parents[1]
MODEL_ROOT = PLUGIN_ROOT / "models"
ENVIRONMENT_MODELS = MODEL_ROOT / "environments"
SCAFFOLD_ROOT = PLUGIN_ROOT / "scaffold" / "sim_bringup"

STARTUP_TIMEOUT_SECONDS = 240
TEST_TIMEOUT_SECONDS = 600
MAX_TEST_TIMEOUT_SECONDS = 1800
SHUTDOWN_GRACE_SECONDS = 8

FURNITURE_FOOTPRINTS_M = {
    "cardboard_box": (0.60, 0.50),
    "table": (1.70, 1.00),
    "chair": (1.00, 1.00),
}
FURNITURE_MODELS = {
    "cardboard_box": "cardboard_box",
    "table": "Table",
    "chair": "Office Chair",
}
