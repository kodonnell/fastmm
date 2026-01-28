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
            network.finalize()

    def test_build_rtree_success(self):
        """Test successful rtree index building."""
        network = fastmm.Network()
        network.add_edge(1, source=1, target=2, geom=[(0, 0), (100, 0)])
        network.finalize()  # Should not raise


class TestRtreeIndexRequired:
    """Test that operations fail when rtree index is not built."""

    def test_matching_without_rtree_fails(self):
        """Test that matching fails if finalize() is not called."""
        network = fastmm.Network()
        network.add_edge(1, source=1, target=2, geom=[(0, 0), (100, 0)], speed=50.0)
        # Don't call finalize()

        # This should raise because rtree index was not built
        with pytest.raises(ValueError, match="must be finalized"):
            _ = fastmm.FastMapMatch(
                network, fastmm.TransitionMode.SHORTEST, max_distance_between_candidates=1000, cache_dir=".cache"
            )


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

    def test_trajectory_with_decreasing_timestamps_fails(self):
        """Test that creating trajectory with non-increasing timestamps fails."""
        with pytest.raises(ValueError, match="non-decreasing"):
            fastmm.Trajectory.from_xyt_tuples(1, [(0, 0, 10), (100, 0, 5), (200, 0, 15)])

    def test_trajectory_with_equal_timestamps_succeeds(self):
        """Test that trajectory with equal consecutive timestamps is allowed."""
        # Non-decreasing means t[i] <= t[i+1], so equal timestamps are OK
        traj = fastmm.Trajectory.from_xyt_tuples(1, [(0, 0, 10), (100, 0, 10), (200, 0, 15)])
        assert traj.id == 1
        assert len(traj) == 3


class TestNetworkGraph:
    """Test network graph creation with different routing modes."""

    def test_create_graph_shortest_mode(self):
        """Test creating graph with SHORTEST mode (default)."""
        network = fastmm.Network()
        network.add_edge(1, source=1, target=2, geom=[(0, 0), (100, 0)])
        network.finalize()
        graph = fastmm.NetworkGraph(network, fastmm.TransitionMode.SHORTEST)
        assert graph is not None

    def test_create_graph_fastest_mode_with_speed(self):
        """Test creating graph with FASTEST mode when edges have speed."""
        network = fastmm.Network()
        network.add_edge(1, source=1, target=2, geom=[(0, 0), (100, 0)], speed=50.0)
        network.add_edge(2, source=2, target=3, geom=[(100, 0), (200, 0)], speed=60.0)
        network.finalize()
        graph = fastmm.NetworkGraph(network, fastmm.TransitionMode.FASTEST)
        assert graph is not None

    def test_create_graph_fastest_mode_without_speed_fails(self):
        """Test that FASTEST mode fails if any edge lacks speed."""
        network = fastmm.Network()
        network.add_edge(1, source=1, target=2, geom=[(0, 0), (100, 0)], speed=50.0)
        network.add_edge(2, source=2, target=3, geom=[(100, 0), (200, 0)])  # No speed
        network.finalize()
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
        network.finalize()
        return network

    def test_shortest_routing_prefers_direct_route(self, network_with_detour):
        """Test that SHORTEST routing prefers the direct route (Edge 2)."""
        network = network_with_detour
        matcher = fastmm.FastMapMatch(
            network, fastmm.TransitionMode.SHORTEST, max_distance_between_candidates=5000, cache_dir=".cache"
        )
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
        matcher = fastmm.FastMapMatch(
            network, fastmm.TransitionMode.FASTEST, max_time_between_candidates=5000, cache_dir=".cache"
        )
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
        network.finalize()

        matcher = fastmm.FastMapMatch(
            network,
            fastmm.TransitionMode.SHORTEST,
            max_distance_between_candidates=1000,
            cache_dir=".cache/test_match_result_structure",
        )
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
        network.finalize()

        matcher = fastmm.FastMapMatch(
            network, fastmm.TransitionMode.SHORTEST, max_distance_between_candidates=1000, cache_dir=".cache"
        )
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


class TestSplitMatching:
    """Test automatic trajectory splitting on failures."""

    def test_split_match_basic(self):
        """Test basic split matching with continuous trajectory."""
        network = fastmm.Network()
        network.add_edge(1, source=1, target=2, geom=[(0, 0), (100, 0)])
        network.add_edge(2, source=2, target=3, geom=[(100, 0), (200, 0)])
        network.finalize()

        matcher = fastmm.FastMapMatch(
            network,
            fastmm.TransitionMode.SHORTEST,
            max_distance_between_candidates=1000,
            cache_dir=".cache/test_split_match_result_structure",
        )
        config = fastmm.FastMapMatchConfig(k=4, candidate_search_radius=50, gps_error=50)

        # Simple trajectory that should match completely
        trajectory = fastmm.Trajectory.from_xy_tuples(1, [(10, 0), (50, 0), (150, 0)])
        result = matcher.pymatch_trajectory_split(trajectory, config)

        assert result.id == 1
        assert len(result.subtrajectories) >= 1
        # Should have at least one successful sub-trajectory
        success_count = sum(1 for sub in result.subtrajectories if sub.error_code == fastmm.MatchErrorCode.SUCCESS)
        assert success_count >= 1

    def test_split_match_with_gap(self):
        """Test split matching with a point far from network (simulates failure)."""
        network = fastmm.Network()
        network.add_edge(1, source=1, target=2, geom=[(0, 0), (100, 0)])
        network.add_edge(2, source=2, target=3, geom=[(100, 0), (200, 0)])
        network.finalize()

        matcher = fastmm.FastMapMatch(
            network, fastmm.TransitionMode.SHORTEST, max_distance_between_candidates=1000, cache_dir=".cache"
        )
        config = fastmm.FastMapMatchConfig(k=4, candidate_search_radius=30, gps_error=20)

        # Trajectory with point far from network
        trajectory = fastmm.Trajectory.from_xy_tuples(
            1,
            [
                (10, 0),  # Point 0 - near network
                (50, 0),  # Point 1 - near network
                (100, 500),  # Point 2 - far from network (should be skipped)
                (150, 0),  # Point 3 - near network
                (180, 0),  # Point 4 - near network
            ],
        )
        result = matcher.pymatch_trajectory_split(trajectory, config)

        assert result.id == 1

        # Should have successfully matched portions (points 0-1 and 3-4)
        # Point 2 is just skipped (no single-point sub-trajectories added)
        assert len(result.subtrajectories) >= 1

        # All returned sub-trajectories should be successful
        for sub in result.subtrajectories:
            assert sub.error_code == fastmm.MatchErrorCode.SUCCESS

        # Print results for debugging
        print(f"\nSplit match result: {len(result.subtrajectories)} sub-trajectories")
        for i, sub in enumerate(result.subtrajectories):
            print(f"  Sub {i}: points [{sub.start_index}-{sub.end_index}] with {len(sub.segments)} segments")

    def test_split_match_result_structure(self):
        """Test the structure of split match results."""
        network = fastmm.Network()
        network.add_edge(1, source=1, target=2, geom=[(0, 0), (100, 0)])
        network.finalize()

        matcher = fastmm.FastMapMatch(
            network, fastmm.TransitionMode.SHORTEST, max_distance_between_candidates=1000, cache_dir=".cache"
        )
        config = fastmm.FastMapMatchConfig(k=4, candidate_search_radius=50, gps_error=50)
        trajectory = fastmm.Trajectory.from_xy_tuples(1, [(10, 0), (50, 0)])

        result = matcher.pymatch_trajectory_split(trajectory, config)

        # Check result structure
        assert hasattr(result, "id")
        assert hasattr(result, "subtrajectories")
        assert isinstance(result.subtrajectories, list)

        # Check sub-trajectory structure
        for sub in result.subtrajectories:
            assert hasattr(sub, "start_index")
            assert hasattr(sub, "end_index")
            assert hasattr(sub, "error_code")
            assert hasattr(sub, "segments")
            assert isinstance(sub.segments, list)

            # All returned sub-trajectories should be successful
            assert sub.error_code == fastmm.MatchErrorCode.SUCCESS

            # For 2+ points, should have at least 1 segment
            if sub.end_index > sub.start_index:
                assert len(sub.segments) >= 1

    def test_split_match_disconnected_by_distance(self):
        """Test split matching when points are too far apart for UBODT (disconnected layers)."""
        # Create a long network
        network = fastmm.Network()
        network.add_edge(1, source=1, target=2, geom=[(0, 0), (100, 0)])
        network.add_edge(2, source=2, target=3, geom=[(100, 0), (200, 0)])
        network.add_edge(3, source=3, target=4, geom=[(200, 0), (300, 0)])
        network.add_edge(4, source=4, target=5, geom=[(300, 0), (400, 0)])
        network.add_edge(5, source=5, target=6, geom=[(400, 0), (500, 0)])
        network.add_edge(6, source=6, target=7, geom=[(500, 0), (600, 0)])
        network.add_edge(7, source=7, target=8, geom=[(600, 0), (700, 0)])
        network.add_edge(8, source=8, target=9, geom=[(700, 0), (800, 0)])
        network.finalize()

        matcher = fastmm.FastMapMatch(
            network, fastmm.TransitionMode.SHORTEST, max_distance_between_candidates=250, cache_dir=".cache"
        )
        # Use k=1 and small radius to ensure we get the closest edge only
        config = fastmm.FastMapMatchConfig(k=1, candidate_search_radius=30, gps_error=50)

        # First, test with regular matching to see what happens
        test_traj = fastmm.Trajectory.from_xy_tuples(
            0,
            [
                (150, 0),  # Point on edge 2 (middle of 100-200)
                (750, 0),  # Point on edge 8 (middle of 700-800) - very far away
            ],
        )
        test_result = matcher.pymatch_trajectory(test_traj, config)
        print(f"\nRegular match result (should fail with DISCONNECTED_LAYERS): {test_result.error_code}")

        # Create trajectory with points that:
        # - All have candidates (near roads)
        # - But some consecutive points are beyond UBODT delta distance
        trajectory = fastmm.Trajectory.from_xy_tuples(
            1,
            [
                (50, 0),  # Point 0 - on edge 1 (middle of 0-100)
                (150, 0),  # Point 1 - on edge 2 (middle of 100-200)
                (750, 0),  # Point 2 - on edge 8 (middle of 700-800) - path requires >500 units
                (780, 0),  # Point 3 - also on edge 8
            ],
        )
        result = matcher.pymatch_trajectory_split(trajectory, config)

        assert result.id == 1

        # Should split into at least 2 sub-trajectories due to disconnection
        # Point 1 (edge 2) to Point 2 (edge 7) requires path distance of ~400-500 units
        # which exceeds delta=250, so UBODT won't have this path precomputed
        # Expected: Points 0-1 match, then disconnect at 1->2, then points 2-3 match
        assert len(result.subtrajectories) >= 2

        # All returned sub-trajectories should be successful
        for sub in result.subtrajectories:
            assert sub.error_code == fastmm.MatchErrorCode.SUCCESS

        # Print results for debugging
        print(f"\nDisconnected layers test: {len(result.subtrajectories)} sub-trajectories")
        for i, sub in enumerate(result.subtrajectories):
            num_points = sub.end_index - sub.start_index + 1
            print(
                f"  Sub {i}: points [{sub.start_index}-{sub.end_index}] ({num_points} points) with {len(sub.segments)} segments"
            )


if __name__ == "__main__":
    # Allow running without pytest
    print("Running tests...")
    pytest.main([__file__, "-v"])
