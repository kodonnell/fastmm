import logging
from pathlib import Path

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)

import fmm


class MapMatcher:
    """High-level map matcher that handles matching failures and time interpolation.

    This class wraps around the low-level FMM matcher and provides automatic
    handling of matching failures by splitting trajectories and interpolating
    timestamps onto matched points.
    """

    def __init__(self, network: fmm.Network, ubodt_max_djikstra_distance: float, cache_dir: Path):
        self.network = network
        logger.info(f"Network with {self.network.get_node_count()} nodes and {self.network.get_edge_count()} edges")
        self.ubodt_max_djikstra_distance = ubodt_max_djikstra_distance
        self.cache_dir = Path(cache_dir)
        logger.info("Creating the network graph")
        self.graph = fmm.NetworkGraph(self.network)
        ubodt_path = self.cache_dir / f"ubodt-{self.ubodt_max_djikstra_distance}.bin"
        if not ubodt_path.exists():
            logger.info(f"Generating UBODT and saving to {ubodt_path}")
            ubodt = fmm.UBODTGenAlgorithm(self.network, self.graph)
            ubodt.generate_ubodt(str(ubodt_path), self.ubodt_max_djikstra_distance)
        logger.info(f"Loading UBODT from {ubodt_path}")
        self.ubodt = fmm.UBODT.read_ubodt_binary(str(ubodt_path), 50000)
        self.model = fmm.FastMapMatch(self.network, self.graph, self.ubodt)
