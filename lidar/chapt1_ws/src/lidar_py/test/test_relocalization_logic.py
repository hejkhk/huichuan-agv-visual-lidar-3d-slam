"""Focused tests for immutable and multi-frame relocalization decisions."""

import math

import pytest

from lidar_py.relocalization_logic import (
    ImmutableCrcLock,
    PoseConsensus,
    occupancy_grid_crc,
    refine_distinct_candidates,
)


def test_second_best_is_refined_before_margin_is_computed():
    coarse = [
        (0.80, 10, 10, 0.0),
        (0.73, 70, 70, math.pi / 2),
    ]

    def refine(seed):
        if seed[1] == 70:
            return (0.79, seed[1], seed[2], seed[3])
        return (0.81, seed[1], seed[2], seed[3])

    ranked = refine_distinct_candidates(
        coarse, refine, 0.05, 0.8, math.radians(20.0), 8)

    assert [candidate[0] for candidate in ranked] == [0.81, 0.79]
    assert ranked[0][0] - ranked[1][0] == pytest.approx(0.02)


def test_three_distinct_consistent_scans_reach_consensus():
    consensus = PoseConsensus(3, 0.35, math.radians(12.0))

    assert not consensus.observe(1.00, 2.00, 0.10, 100).ready
    assert not consensus.observe(1.05, 1.98, 0.12, 200).ready
    result = consensus.observe(0.98, 2.02, 0.09, 300)

    assert result.ready
    assert result.count == 3
    assert result.pose[0] == pytest.approx(1.01, abs=0.02)


def test_candidate_jump_resets_consensus():
    consensus = PoseConsensus(3, 0.35, math.radians(12.0))
    consensus.observe(1.0, 2.0, 0.0, 100)
    consensus.observe(1.1, 2.0, 0.02, 200)

    result = consensus.observe(4.0, -1.0, math.pi / 2, 300)

    assert result.reset
    assert result.count == 1
    assert not result.ready


def test_duplicate_scan_timestamp_does_not_increment_consensus():
    consensus = PoseConsensus(3, 0.35, math.radians(12.0))
    consensus.observe(1.0, 2.0, 0.0, 100)

    result = consensus.observe(1.0, 2.0, 0.0, 100)

    assert result.duplicate
    assert result.count == 1


def test_reference_crc_changes_for_cells_or_metadata():
    origin = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    base = occupancy_grid_crc(2, 2, 0.05, origin, [0, 100, -1, 0])

    assert base == occupancy_grid_crc(2, 2, 0.05, origin, [0, 100, -1, 0])
    assert base != occupancy_grid_crc(2, 2, 0.05, origin, [0, 100, -1, 1])
    assert base != occupancy_grid_crc(2, 2, 0.10, origin, [0, 100, -1, 0])


def test_reference_crc_lock_accepts_republish_and_rejects_mutation():
    lock = ImmutableCrcLock()

    assert lock.accept(0x12345678)
    assert lock.accept(0x12345678)
    assert not lock.accept(0x87654321)
    assert lock.crc == 0x12345678
