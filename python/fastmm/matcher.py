import logging
from pathlib import Path
from typing import Optional

import fastmm

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)


class MapMatcher:
    """High-level map matcher with automatic UBODT caching and mode validation.

    This class wraps the low-level FastMM matcher and provides:
    - Automatic UBODT generation and caching based on network and mode
    - Mode validation (ensures UBODT matches graph mode)
    - Simplified API with clear parameter names

    Example:
        # SHORTEST mode (distance-based)
        network = fastmm.Network()
        # ... add edges ...
        network.build_rtree_index()

        matcher = MapMatcher(
            network=network,
            mode=fastmm.TransitionMode.SHORTEST,
            max_distance_between_candidates=300.0,  # meters
            cache_dir="./cache"
        )

        # FASTEST mode (time-based)
        matcher = MapMatcher(
            network=network,
            mode=fastmm.TransitionMode.FASTEST,
            max_time_between_candidates=20.0,  # seconds
            cache_dir="./cache"
        )
    """

    def __init__(
        self,
        network: fastmm.Network,
        mode: fastmm.TransitionMode = fastmm.TransitionMode.SHORTEST,
        max_distance_between_candidates: Optional[float] = None,
        max_time_between_candidates: Optional[float] = None,
        cache_dir: Optional[Path] = None,
    ):
        """Initialize the MapMatcher.

        Args:
            network: Road network with edges and rtree index built
            mode: Routing mode (SHORTEST for distance-based, FASTEST for time-based)
            max_distance_between_candidates: Maximum distance in meters for UBODT delta (SHORTEST mode only)
            max_time_between_candidates: Maximum time in seconds for UBODT delta (FASTEST mode only)
            cache_dir: Directory for caching UBODT files (default: ./ubodt_cache)

        Raises:
            ValueError: If mode/delta parameter mismatch or network incompatible with mode
        """
        self.network = network
        self.mode = mode

        # Validate and set delta based on mode
        if mode == fastmm.TransitionMode.SHORTEST:
            if max_distance_between_candidates is None:
                raise ValueError("max_distance_between_candidates required for SHORTEST mode")
            if max_time_between_candidates is not None:
                logger.warning("max_time_between_candidates ignored in SHORTEST mode")
            delta = max_distance_between_candidates
            mode_name = "shortest"
        elif mode == fastmm.TransitionMode.FASTEST:
            if max_time_between_candidates is None:
                raise ValueError("max_time_between_candidates required for FASTEST mode")
            if max_distance_between_candidates is not None:
                logger.warning("max_distance_between_candidates ignored in FASTEST mode")
            delta = max_time_between_candidates
            mode_name = "fastest"
        else:
            raise ValueError(f"Unknown mode: {mode}")

        logger.info(f"Network with {network.get_node_count()} nodes and {network.get_edge_count()} edges")
        logger.info(f"Mode: {mode_name.upper()}, delta: {delta}")

        # Set up cache directory
        cache_dir_path = Path(cache_dir) if cache_dir else Path("./ubodt_cache")

        # Create the matcher - it handles all UBODT management internally
        self.model = fastmm.FastMapMatch(network, mode, delta, str(cache_dir_path))
        logger.info("MapMatcher initialized successfully")
