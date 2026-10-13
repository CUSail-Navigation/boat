"""Numpy-only inference for a trained PPO waypoint policy.

This file depends on numpy alone (PyYAML optionally) so it can be copied as-is
into a ROS package and run on the boat without stable-baselines3 or torch.

It loads either:
- an SB3 checkpoint zip directly (e.g. ``runs/<run>/best_model/best_model.zip``), or
- an ``.npz`` written by ``scripts/export_policy_numpy.py``.
"""

from __future__ import annotations

import io
import json
import pickle
import re
import zipfile
from collections import OrderedDict
from pathlib import Path
from typing import TYPE_CHECKING, Any

import numpy as np

if TYPE_CHECKING:
    from collections.abc import Callable

    from numpy.typing import NDArray

_ACTIVATIONS: dict[str, Callable[[NDArray[np.float32]], NDArray[np.float32]]] = {
    "tanh": np.tanh,
    "relu": lambda x: np.maximum(x, 0.0),
}

# torch storage classes that can appear in a saved state dict, as numpy dtypes.
_TORCH_STORAGE_DTYPES = {
    "FloatStorage": np.float32,
    "DoubleStorage": np.float64,
    "HalfStorage": np.float16,
    "LongStorage": np.int64,
    "IntStorage": np.int32,
    "BoolStorage": np.bool_,
}


class NumpyPolicy:
    """Deterministic MLP actor: hidden layers -> activation, then a linear action head."""

    def __init__(self, path: str | Path, activation: str | None = None) -> None:
        """Load a policy from an SB3 ``.zip`` checkpoint or an exported ``.npz``.

        Args:
            path: Path to ``best_model.zip`` (or any SB3 PPO checkpoint) or an ``.npz``.
            activation: Hidden-layer activation for a ``.zip`` trained with a
                non-default ``activation_fn``. Defaults to SB3's ``tanh``.

        """
        path = Path(path)
        # Observation scales the policy was trained with (from WaypointEnvConfig).
        self.env_config: dict[str, float] = {}
        if path.suffix == ".zip":
            self._load_sb3_zip(path, activation)
        else:
            self._load_npz(path)

    def _load_npz(self, path: Path) -> None:
        with np.load(path) as data:
            n_layers = int(data["n_layers"])
            self.weights: list[NDArray[np.float32]] = [data[f"w{i}"].astype(np.float32) for i in range(n_layers)]
            self.biases: list[NDArray[np.float32]] = [data[f"b{i}"].astype(np.float32) for i in range(n_layers)]
            self.activation = _ACTIVATIONS[str(data["activation"])]
            self.env_config = {
                key.removeprefix("cfg_"): float(data[key]) for key in data.files if key.startswith("cfg_")
            }

    def _load_sb3_zip(self, path: Path, activation: str | None) -> None:
        with zipfile.ZipFile(path) as checkpoint:
            metadata = json.loads(checkpoint.read("data"))
            state_dict = _load_torch_state_dict(checkpoint.read("policy.pth"))

        # A custom activation_fn is stored cloudpickled; it can't be read without torch.
        if activation is None:
            if "activation_fn" in json.dumps(metadata.get("policy_kwargs", {})):
                msg = "Checkpoint uses a custom activation_fn; pass activation= explicitly."
                raise ValueError(msg)
            activation = "tanh"
        self.activation = _ACTIVATIONS[activation]

        # Actor = mlp_extractor.policy_net.<i> Linear layers in order, then action_net.
        # (value_net and log_std are only used during training.)
        indices = sorted(
            int(m.group(1))
            for key in state_dict
            if (m := re.fullmatch(r"mlp_extractor\.policy_net\.(\d+)\.weight", key))
        )
        prefixes = [f"mlp_extractor.policy_net.{i}" for i in indices] + ["action_net"]
        self.weights = [state_dict[f"{p}.weight"].astype(np.float32) for p in prefixes]
        self.biases = [state_dict[f"{p}.bias"].astype(np.float32) for p in prefixes]

        self.env_config = _load_env_config(path)

    @property
    def obs_dim(self) -> int:
        """Length of the observation vector the policy expects."""
        return int(self.weights[0].shape[1])

    def predict(self, obs: NDArray[np.float32]) -> NDArray[np.float32]:
        """Return the action in [-1, 1] for a single observation vector."""
        h = np.asarray(obs, dtype=np.float32)
        for w, b in zip(self.weights[:-1], self.biases[:-1], strict=True):
            h = self.activation(w @ h + b)
        action = self.weights[-1] @ h + self.biases[-1]
        return np.clip(action, -1.0, 1.0).astype(np.float32)


class _StorageType:
    """Stand-in for a torch storage class; carries the numpy dtype."""

    def __init__(self, dtype: type[np.generic]) -> None:
        self.dtype = np.dtype(dtype).newbyteorder("<")


def _rebuild_tensor(
    storage: NDArray[Any],
    storage_offset: int,
    size: tuple[int, ...],
    stride: tuple[int, ...],
    *_args: object,
) -> NDArray[Any]:
    """Numpy equivalent of torch._utils._rebuild_tensor_v2."""
    if not size:
        return np.array(storage[storage_offset])
    itemsize = storage.itemsize
    return np.lib.stride_tricks.as_strided(
        storage[storage_offset:],
        shape=size,
        strides=tuple(s * itemsize for s in stride),
    ).copy()


class _TorchStateDictUnpickler(pickle.Unpickler):
    """Unpickle a torch.save()'d state dict into numpy arrays, without torch.

    Only the handful of globals a plain tensor state dict uses are allowed, so an
    unexpected checkpoint fails loudly instead of executing arbitrary code.
    """

    def __init__(self, data: bytes, archive: zipfile.ZipFile, prefix: str) -> None:
        super().__init__(io.BytesIO(data))
        self.archive = archive
        self.prefix = prefix

    def find_class(self, module: str, name: str) -> Any:  # noqa: ANN401 - pickle API
        if module == "collections" and name == "OrderedDict":
            return OrderedDict
        if module == "torch._utils" and name == "_rebuild_tensor_v2":
            return _rebuild_tensor
        if module == "torch" and name in _TORCH_STORAGE_DTYPES:
            return _StorageType(_TORCH_STORAGE_DTYPES[name])
        msg = f"Unsupported global in checkpoint: {module}.{name}"
        raise pickle.UnpicklingError(msg)

    def persistent_load(self, pid: tuple[str, _StorageType, str, str, int]) -> NDArray[Any]:
        # Each tensor's bytes live in <prefix>/data/<key>; pid carries the key and dtype.
        _, storage_type, key, _location, _numel = pid
        raw = self.archive.read(f"{self.prefix}/data/{key}")
        return np.frombuffer(raw, dtype=storage_type.dtype)


def _load_torch_state_dict(blob: bytes) -> dict[str, NDArray[Any]]:
    """Read a torch.save() zip archive (as bytes) into a dict of numpy arrays."""
    with zipfile.ZipFile(io.BytesIO(blob)) as archive:
        pkl_name = next(n for n in archive.namelist() if n.endswith("data.pkl"))
        prefix = pkl_name.rsplit("/", 1)[0]
        byteorder_name = f"{prefix}/byteorder"
        if byteorder_name in archive.namelist() and archive.read(byteorder_name) != b"little":
            msg = "Only little-endian checkpoints are supported."
            raise ValueError(msg)
        state_dict = _TorchStateDictUnpickler(archive.read(pkl_name), archive, prefix).load()
    return dict(state_dict)


def _load_env_config(model_path: Path) -> dict[str, float]:
    """Read numeric env settings from the run's config_used.yaml, if present.

    Checkpoints live at runs/<run>/best_model/*.zip or runs/<run>/checkpoints/*.zip,
    so the config is one or two directories up.
    """
    for candidate in (model_path.parent / "config_used.yaml", model_path.parent.parent / "config_used.yaml"):
        if candidate.exists():
            try:
                import yaml  # type: ignore[import-untyped]  # noqa: PLC0415 - optional dependency
            except ImportError:
                return {}
            env = (yaml.safe_load(candidate.read_text(encoding="utf-8")) or {}).get("env", {})
            return {k: float(v) for k, v in env.items() if isinstance(v, (int, float)) and not isinstance(v, bool)}
    return {}
