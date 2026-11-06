"""What the arm believes is on the table, fused over several viewpoints.

Pure standard library: no ROS, no numpy.  `VisionFrontend` feeds measurements in,
`WorldScene` renders them as collision objects, and the grasp code reads
the fused positions out.  Keeping the state here is what lets a pick use
"where I first saw it" instead of "where the camera last happened to
point".
"""
import statistics

# rank of an observation taken while looking around rather than from the
# default observation pose
SCAN_RANK = 99


class SceneModel:

    def __init__(self):
        self.seen = {}
        self.first_seen = {}
        self.reference_seen = {}
        self._observations = {}

    def reset(self):
        self.seen.clear()
        self.first_seen.clear()
        self.reference_seen.clear()
        self._observations.clear()

    def add(self, label, point, area, rank):
        """Record one measurement of `label` from one viewpoint."""
        self._observations.setdefault(label, []).append(
            (point, area, rank))

    def set_reference(self, label, point):
        """Position seen from the canonical observation pose."""
        self.reference_seen[label] = point

    def fuse(self):
        """Median of every observation so far, per label."""
        for label, observations in self._observations.items():
            if not observations:
                continue
            xs = [item[0][0] for item in observations]
            ys = [item[0][1] for item in observations]
            zs = [item[0][2] for item in observations]
            self.seen[label] = (float(statistics.median(xs)),
                                float(statistics.median(ys)),
                                float(statistics.median(zs)))
            first = [item for item in observations if item[2] == 0]
            if first and label not in self.first_seen:
                self.first_seen[label] = first[0][0]

    def best(self, label):
        """Reference pose, else first sighting, else the fused median."""
        return (self.reference_seen.get(label) or
                self.first_seen.get(label) or
                self.seen.get(label))

    def labels(self):
        return sorted(self.seen)

    def known(self, labels):
        return all(label in self.seen for label in labels)
