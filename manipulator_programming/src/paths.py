from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
MODELS_DIR = PROJECT_ROOT / "models"
MUJOCO_MODELS_DIR = MODELS_DIR / "mujoco"


def mujoco_model_path(*parts: str) -> Path:
    return MUJOCO_MODELS_DIR.joinpath(*parts)
