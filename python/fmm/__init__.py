# fmm - Fast Map Matching for Python

# Import C++ bindings
from .fmm import *

# Import Python helpers
from .matcher import MapMatcher

try:
    from ._version import version as __version__
except ImportError:
    __version__ = "unknown"

# C++ classes from bindings
__all__ = [
    # Core classes
    "Network",
    "NetworkGraph",
    "FastMapMatch",
    "FastMapMatchConfig",
    "UBODT",
    "UBODTGenAlgorithm",
    "Trajectory",
    # Geometry classes
    "Point",
    "LineString",
    # Result classes
    "PyMatchResult",
    "PyMatchSegment",
    "PyMatchCandidate",
    "PyMatchPoint",
    "PyMatchSegmentEdge",
    "MatchErrorCode",
    # Python helpers
    "MapMatcher",
]
