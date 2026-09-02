"""Dataset manifests: what to evaluate, and what to compare it against."""

import pathlib

import yaml

_REQUIRED = ("dataset", "title", "config", "robots")
_ALIGNMENTS = ("rigid", "sim3", "yaw")


class ManifestError(ValueError):
    pass


class Robot:
    def __init__(self, entry, index):
        if "id" not in entry:
            raise ManifestError(f"robots[{index}] has no id")
        self.id = str(entry["id"])
        self.bag = entry.get("bag")
        self.ground_truth = entry.get("ground_truth")
        self.estimate = entry.get("estimate")
        self.candidates = entry.get("candidates")
        self.registrations = entry.get("registrations")
        self.comms_log = entry.get("comms_log")


class Manifest:
    def __init__(self, document, path=None):
        self.path = path
        missing = [key for key in _REQUIRED if key not in document]
        if missing:
            raise ManifestError(
                f"missing required field(s): {', '.join(missing)}")

        self.dataset = str(document["dataset"])
        self.title = str(document["title"])
        self.description = str(document.get("description", ""))
        self.config = str(document["config"])
        self.launch = document.get("launch")

        robots = document["robots"]
        if not isinstance(robots, list) or not robots:
            raise ManifestError("robots must be a non-empty list")
        self.robots = [Robot(entry, index)
                       for index, entry in enumerate(robots)]

        self.alignment = str(document.get("alignment", "rigid"))
        if self.alignment not in _ALIGNMENTS:
            raise ManifestError(
                f"alignment must be one of {', '.join(_ALIGNMENTS)}, "
                f"got {self.alignment!r}")

        self.association_max_time_difference_s = float(
            document.get("association_max_time_difference_s", 0.02))
        if self.association_max_time_difference_s <= 0.0:
            raise ManifestError(
                "association_max_time_difference_s must be positive")

        self.relative_pose_delta = int(document.get("relative_pose_delta", 1))
        if self.relative_pose_delta < 1:
            raise ManifestError("relative_pose_delta must be at least 1")

        place = self._section(document, "place_recognition")
        self.revisit_distance_m = float(place.get("revisit_distance_m", 10.0))
        self.place_min_time_gap_s = float(place.get("min_time_gap_s", 30.0))
        self.total_positives = place.get("total_positives")

        registration = self._section(document, "registration")
        self.registration_translation_threshold_m = float(
            registration.get("translation_threshold_m", 2.0))
        self.registration_rotation_threshold_deg = float(
            registration.get("rotation_threshold_deg", 5.0))

        resources = self._section(document, "resources")
        self.knn_feature_dim = int(resources.get("knn_feature_dim", 40))
        self.num_sectors = int(resources.get("num_sectors", 60))
        self.scan_context_rings = resources.get("scan_context_rings")

        self.expected = self._section(document, "expected")

    @staticmethod
    def _section(document, name):
        """An optional mapping section, absent or present but never mistyped.

        `key:` with nothing under it is a legitimate way to write an empty
        section; `key: []` is a mistake, and coercing it would hide one.
        """
        value = document.get(name)
        if value is None:
            return {}
        if not isinstance(value, dict):
            raise ManifestError(
                f"{name} must be a mapping, got {type(value).__name__}")
        return value

    def resolve(self, relative, root=None):
        """Resolve a manifest-relative path against `root` or the manifest."""
        if relative is None:
            return None
        candidate = pathlib.Path(relative)
        if candidate.is_absolute():
            return candidate
        base = (pathlib.Path(root) if root is not None
                else (self.path.parent if self.path else pathlib.Path(".")))
        return base / candidate


def load_manifest(path):
    path = pathlib.Path(path)
    try:
        document = yaml.safe_load(path.read_text())
    except yaml.YAMLError as error:
        raise ManifestError(f"{path}: not valid YAML: {error}") from error
    if not isinstance(document, dict):
        raise ManifestError(f"{path}: expected a mapping at the top level")
    try:
        return Manifest(document, path=path)
    except ManifestError as error:
        raise ManifestError(f"{path}: {error}") from error
