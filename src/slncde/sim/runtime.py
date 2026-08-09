import importlib
import sys
import types
from pathlib import Path


def load_simulator(repo_root: Path):
    simulator_root = repo_root / "external" / "deformable-ravens"
    if str(simulator_root) not in sys.path:
        sys.path.insert(0, str(simulator_root))
    if "ravens" not in sys.modules:
        package = types.ModuleType("ravens")
        package.__path__ = [str(simulator_root / "ravens")]
        package.__package__ = "ravens"
        sys.modules["ravens"] = package
    tasks = importlib.import_module("ravens.tasks")
    Environment = importlib.import_module("ravens.environment").Environment
    return simulator_root, tasks, Environment
