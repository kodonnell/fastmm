"""
Comprehensive tests for fastmm Python bindings.

Run with: pytest test_fastmm.py -v
Or: python test_fastmm.py
"""

import sys
from pathlib import Path

# For running without installation
sys.path.insert(0, "build/python/pybind11/Release")

import fastmm
import pytest


class TestNetworkBasics:
    """Test basic network creation and operations."""

    def test_create_empty_network(self):
        """Test creating an empty network."""
        network = fastmm.Network()
        assert network.get_edge_count() == 0
        assert network.get_node_count() == 0

    def test_add_edge_with_coordinates(self):
        """Test adding edges using coordinate lists."""
        network = fastmm.Network()
        network.add_edge(1, source=1, target=2, geom=[(0, 0), (100, 0)])
        assert network.get_edge_count() == 1
        assert network.get_node_count() == 2

    def test_add_edge_with_speed(self):
        """Test adding edges with speed values."""
        network = fastmm.Network()
        network.add_edge(1, source=1, target=2, geom=[(0, 0), (100, 0)], speed=50.0)
        assert network.get_edge_count() == 1

    def test_add_multiple_edges(self):
        """Test adding multiple edges to build a network."""
        network = fastmm.Network()
        network.add_edge(1, source=1, target=2, geom=[(0, 0), (100, 0)])
        network.add_edge(2, source=2, target=3, geom=[(100, 0), (200, 0)])
        network.add_edge(3, source=2, target=4, geom=[(100, 0), (100, 100)])
        assert network.get_edge_count() == 3
        assert network.get_node_count() == 4  # Nodes: 1, 2, 3, 4

    def test_add_edge_minimum_points(self):
        """Test that edges require at least 2 points."""
        network = fastmm.Network()
        with pytest.raises(RuntimeError, match="at least 2 points"):
            network.add_edge(1, source=1, target=2, geom=[(0, 0)])

    def test_add_edge_empty_geometry(self):
        """Test that edges require non-empty geometry."""
        network = fastmm.Network()
        with pytest.raises(RuntimeError, match="at least 2 points"):
            network.add_edge(1, source=1, target=2, geom=[])

    def test_add_edge_invalid_coordinate_format(self):
        """Test that coordinates must be tuples of (x, y)."""
        network = fastmm.Network()
        with pytest.raises(RuntimeError, match="must be a tuple"):
            network.add_edge(1, source=1, target=2, geom=[(0, 0, 0)])  # 3D not allowed

    def test_build_rtree_on_empty_network(self):
        """Test that building rtree on empty network fails."""
        network = fastmm.Network()
        with pytest.raises(RuntimeError, match="empty network"):
            network.build_rtree_index()

    def test_build_rtree_success(self):
        """Test successful rtree index building."""
        network = fastmm.Network()
        network.add_edge(1, source=1, target=2, geom=[(0, 0), (100, 0)])
        network.build_rtree_index()  # Should not raise


class TestRtreeIndexRequired:
    """Test that operations fail when rtree index is not built."""

    def test_matching_without_rtree_fails(self):
        """Test that matching fails if build_rtree_index() is not called."""
        network = fastmm.Network()
        network.add_edge(1, source=1, target=2, geom=[(0, 0), (100, 0)], speed=50.0)
        # Don't call build_rtree_index()

        graph = fastmm.NetworkGraph(network, fastmm.TransitionMode.SHORTEST)
        cache_dir = Path("./.cache")
        cache_dir.mkdir(exist_ok=True)
        ubodt_path = cache_dir / "test_ubodt.bin"

        ubodt_gen = fastmm.UBODTGenAlgorithm(network, graph)
        ubodt_gen.generate_ubodt(str(ubodt_path), delta=1000)
        ubodt = fastmm.UBODT.read_ubodt_binary(str(ubodt_path), multiplier=5000)

        matcher = fastmm.FastMapMatch(network, graph, ubodt)
        config = fastmm.FastMapMatchConfig(k=4, candidate_search_radius=50, gps_error=50)
        trajectory = fastmm.Trajectory.from_xy_tuples(1, [(50, 0), (75, 0)])

        # This should raise because rtree index was not built
        with pytest.raises(RuntimeError, match="Spatial index not built"):
            matcher.pymatch_trajectory(trajectory, config)


class TestTrajectoryCreation:
    """Test trajectory creation and manipulation."""

    def test_create_trajectory_from_xy(self):
        """Test creating trajectory from (x, y) tuples."""
        traj = fastmm.Trajectory.from_xy_tuples(1, [(0, 0), (100, 0), (200, 0)])
        assert traj.id == 1
        assert len(traj) == 3

    def test_create_trajectory_from_xyt(self):
        """Test creating trajectory from (x, y, t) tuples."""
        traj = fastmm.Trajectory.from_xyt_tuples(2, [(0, 0, 0), (100, 0, 10), (200, 0, 20)])
        assert traj.id == 2
        assert len(traj) == 3

    def test_trajectory_to_xy_tuples(self):
        """Test exporting trajectory to (x, y) tuples."""
        traj = fastmm.Trajectory.from_xy_tuples(1, [(0, 0), (100, 0)])
        xy_tuples = traj.to_xy_tuples()
        assert len(xy_tuples) == 2
        assert xy_tuples[0] == (0, 0)
        assert xy_tuples[1] == (100, 0)

    def test_trajectory_to_xyt_tuples(self):
        """Test exporting trajectory to (x, y, t) tuples."""
        traj = fastmm.Trajectory.from_xyt_tuples(1, [(0, 0, 5), (100, 0, 10)])
        xyt_tuples = traj.to_xyt_tuples()
        assert len(xyt_tuples) == 2
        assert xyt_tuples[0] == (0, 0, 5)
        assert xyt_tuples[1] == (100, 0, 10)


class TestNetworkGraph:
    """Test network graph creation with different routing modes."""

    def test_create_graph_shortest_mode(self):
        """Test creating graph with SHORTEST mode (default)."""
        network = fastmm.Network()
        network.add_edge(1, source=1, target=2, geom=[(0, 0), (100, 0)])
        network.build_rtree_index()
        graph = fastmm.NetworkGraph(network, fastmm.TransitionMode.SHORTEST)
        assert graph is not None

    def test_create_graph_fastest_mode_with_speed(self):
        """Test creating graph with FASTEST mode when edges have speed."""
        network = fastmm.Network()
        network.add_edge(1, source=1, target=2, geom=[(0, 0), (100, 0)], speed=50.0)
        network.add_edge(2, source=2, target=3, geom=[(100, 0), (200, 0)], speed=60.0)
        network.build_rtree_index()
        graph = fastmm.NetworkGraph(network, fastmm.TransitionMode.FASTEST)
        assert graph is not None

    def test_create_graph_fastest_mode_without_speed_fails(self):
        """Test that FASTEST mode fails if any edge lacks speed."""
        network = fastmm.Network()
        network.add_edge(1, source=1, target=2, geom=[(0, 0), (100, 0)], speed=50.0)
        network.add_edge(2, source=2, target=3, geom=[(100, 0), (200, 0)])  # No speed
        network.build_rtree_index()
        with pytest.raises(ValueError, match="speed"):
            fastmm.NetworkGraph(network, fastmm.TransitionMode.FASTEST)


class TestMatchingConfig:
    """Test map matching configuration."""

    def test_create_config_shortest_mode(self):
        """Test creating config for SHORTEST mode."""
        config = fastmm.FastMapMatchConfig(
            k=8,
            candidate_search_radius=50,
            gps_error=50,
            transition_mode=fastmm.TransitionMode.SHORTEST,
        )
        assert config.k == 8
        assert config.candidate_search_radius == 50
        assert config.gps_error == 50

    def test_create_config_fastest_mode_with_reference_speed(self):
        """Test creating config for FASTEST mode with reference_speed."""
        config = fastmm.FastMapMatchConfig(
            k=8,
            candidate_search_radius=50,
            gps_error=50,
            transition_mode=fastmm.TransitionMode.FASTEST,
            reference_speed=40.0,
        )
        assert config.reference_speed == 40.0

    def test_config_defaults(self):
        """Test configuration default values."""
        config = fastmm.FastMapMatchConfig()
        assert config.k == 8
        assert config.candidate_search_radius == 50
        assert config.gps_error == 50
        assert config.reverse_tolerance == 0.0


class TestShortestVsFastest:
    """Test the example from example_combined.py: shortest vs fastest routing."""

    @pytest.fixture
    def network_with_detour(self):
        """Create the network from example_combined.py with direct and detour routes."""
        network = fastmm.Network()
        # Edge 1: A->B
        network.add_edge(1, source=1, target=2, geom=[(0, 0), (100, 0)], speed=50)
        # Edge 2: B->C direct (slower)
        network.add_edge(2, source=2, target=3, geom=[(100, 0), (200, 0)], speed=50)
        # Edge 3: B->up (faster detour)
        network.add_edge(3, source=2, target=4, geom=[(100, 0), (100, 10)], speed=100)
        # Edge 4: across top (faster detour)
        network.add_edge(4, source=4, target=5, geom=[(100, 10), (200, 10)], speed=100)
        # Edge 5: down->C (faster detour)
        network.add_edge(5, source=5, target=3, geom=[(200, 10), (200, 0)], speed=100)
        # Edge 6: C->D
        network.add_edge(6, source=3, target=6, geom=[(200, 0), (300, 0)], speed=50)
        network.build_rtree_index()
        return network

    def test_shortest_routing_prefers_direct_route(self, network_with_detour):
        """Test that SHORTEST routing prefers the direct route (Edge 2)."""
        network = network_with_detour
        graph = fastmm.NetworkGraph(network, fastmm.TransitionMode.SHORTEST)

        cache_dir = Path("./.cache")
        cache_dir.mkdir(exist_ok=True)
        ubodt_path = cache_dir / "test_shortest.bin"

        ubodt_gen = fastmm.UBODTGenAlgorithm(network, graph)
        ubodt_gen.generate_ubodt(str(ubodt_path), delta=5000)
        ubodt = fastmm.UBODT.read_ubodt_binary(str(ubodt_path), multiplier=50000)

        matcher = fastmm.FastMapMatch(network, graph, ubodt)
        config = fastmm.FastMapMatchConfig(
            k=8,
            candidate_search_radius=15,
            gps_error=5,
            transition_mode=fastmm.TransitionMode.SHORTEST,
        )

        # Trajectory through A->B->?->C->D
        trajectory = fastmm.Trajectory.from_xyt_tuples(
            1, [(25, 0, 0), (75, 0, 5), (150, 5, 10), (225, 0, 15), (275, 0, 20)]
        )

        result = matcher.pymatch_trajectory(trajectory, config)
        assert result.error_code == fastmm.MatchErrorCode.SUCCESS

        # Extract route
        all_edges = []
        for segment in result.segments:
            for edge in segment.edges:
                all_edges.append(edge.edge_id)

        # Remove consecutive duplicates
        route = []
        for edge_id in all_edges:
            if not route or route[-1] != edge_id:
                route.append(edge_id)

        # SHORTEST should prefer direct route: 1 -> 2 -> 6
        assert route == [1, 2, 6], f"Expected [1, 2, 6] but got {route}"

    def test_fastest_routing_prefers_detour_route(self, network_with_detour):
        """Test that FASTEST routing prefers the faster detour route (Edges 3->4->5)."""
        network = network_with_detour
        graph = fastmm.NetworkGraph(network, fastmm.TransitionMode.FASTEST)

        cache_dir = Path("./.cache")
        cache_dir.mkdir(exist_ok=True)
        ubodt_path = cache_dir / "test_fastest.bin"

        ubodt_gen = fastmm.UBODTGenAlgorithm(network, graph)
        ubodt_gen.generate_ubodt(str(ubodt_path), delta=5000)
        ubodt = fastmm.UBODT.read_ubodt_binary(str(ubodt_path), multiplier=50000)

        matcher = fastmm.FastMapMatch(network, graph, ubodt)
        config = fastmm.FastMapMatchConfig(
            k=8,
            candidate_search_radius=15,
            gps_error=5,
            transition_mode=fastmm.TransitionMode.FASTEST,
            reference_speed=50,
        )

        # Trajectory through A->B->?->C->D
        trajectory = fastmm.Trajectory.from_xyt_tuples(
            1, [(25, 0, 0), (75, 0, 5), (150, 5, 10), (225, 0, 15), (275, 0, 20)]
        )

        result = matcher.pymatch_trajectory(trajectory, config)
        assert result.error_code == fastmm.MatchErrorCode.SUCCESS

        # Extract route
        all_edges = []
        for segment in result.segments:
            for edge in segment.edges:
                all_edges.append(edge.edge_id)

        # Remove consecutive duplicates
        route = []
        for edge_id in all_edges:
            if not route or route[-1] != edge_id:
                route.append(edge_id)

        # FASTEST should prefer detour route: 1 -> 3 -> 4 -> 5 -> 6
        assert route == [1, 3, 4, 5, 6], f"Expected [1, 3, 4, 5, 6] but got {route}"


class TestMatchResult:
    """Test match result structure and access."""

    def test_match_result_structure(self):
        """Test that match result has expected structure."""
        network = fastmm.Network()
        network.add_edge(1, source=1, target=2, geom=[(0, 0), (100, 0)], speed=50)
        network.build_rtree_index()

        graph = fastmm.NetworkGraph(network)
        cache_dir = Path("./.cache")
        cache_dir.mkdir(exist_ok=True)
        ubodt_path = cache_dir / "test_result.bin"

        ubodt_gen = fastmm.UBODTGenAlgorithm(network, graph)
        ubodt_gen.generate_ubodt(str(ubodt_path), delta=1000)
        ubodt = fastmm.UBODT.read_ubodt_binary(str(ubodt_path), multiplier=5000)

        matcher = fastmm.FastMapMatch(network, graph, ubodt)
        config = fastmm.FastMapMatchConfig(k=4, candidate_search_radius=50, gps_error=50)
        trajectory = fastmm.Trajectory.from_xy_tuples(1, [(10, 0), (90, 0)])

        result = matcher.pymatch_trajectory(trajectory, config)

        # Check result structure
        assert hasattr(result, "id")
        assert hasattr(result, "error_code")
        assert hasattr(result, "segments")
        assert hasattr(result, "unmatched_candidate_indices")
        assert result.id == 1

    def test_match_result_segments(self):
        """Test accessing match result segments and edges."""
        network = fastmm.Network()
        network.add_edge(1, source=1, target=2, geom=[(0, 0), (100, 0)], speed=50)
        network.add_edge(2, source=2, target=3, geom=[(100, 0), (200, 0)], speed=50)
        network.build_rtree_index()

        graph = fastmm.NetworkGraph(network)
        cache_dir = Path("./.cache")
        cache_dir.mkdir(exist_ok=True)
        ubodt_path = cache_dir / "test_segments.bin"

        ubodt_gen = fastmm.UBODTGenAlgorithm(network, graph)
        ubodt_gen.generate_ubodt(str(ubodt_path), delta=1000)
        ubodt = fastmm.UBODT.read_ubodt_binary(str(ubodt_path), multiplier=5000)

        matcher = fastmm.FastMapMatch(network, graph, ubodt)
        config = fastmm.FastMapMatchConfig(k=4, candidate_search_radius=50, gps_error=50)
        trajectory = fastmm.Trajectory.from_xy_tuples(1, [(10, 0), (150, 0)])

        result = matcher.pymatch_trajectory(trajectory, config)
        assert result.error_code == fastmm.MatchErrorCode.SUCCESS

        # Access segments
        for segment in result.segments:
            assert hasattr(segment, "p0")
            assert hasattr(segment, "p1")
            assert hasattr(segment, "edges")
            for edge in segment.edges:
                assert hasattr(edge, "edge_id")
                assert hasattr(edge, "points")


if __name__ == "__main__":
    # Allow running without pytest
    print("Running tests...")
    pytest.main([__file__, "-v"])
