from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parent.parent


class Cfg(dict):
    """dict with attribute access; nested dicts are converted once, so edits stick."""
    def __init__(self, data=()):
        super().__init__(data)
        for k, v in self.items():
            if isinstance(v, dict) and not isinstance(v, Cfg):
                self[k] = Cfg(v)

    def __getattr__(self, k):
        try:
            return self[k]
        except KeyError as e:
            raise AttributeError(k) from e


def load(path=None, **overrides):
    with open(path or ROOT / "config.yaml") as f:
        data = yaml.safe_load(f)
    for dotted, value in overrides.items():
        node = data
        *parents, leaf = dotted.split(".")
        for p in parents:
            node = node[p]
        node[leaf] = value
    return Cfg(data)
