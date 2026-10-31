"""
WRO FE 2026 - Minimal Pillar-Movement Sequencing Tool
========================================================
Given a set of named pillar configurations (M1, M2, ... read from
pillar_configs_input.txt), this script computes the "movement cost"
between every pair of configurations and finds the ordering that
minimizes the TOTAL number of pillar movements needed to go from one
field setup to the next, across the whole sequence.

MOVEMENT COST DEFINITION (per seat, comparing config A -> config B):
  - Seat empty in A, occupied in B      -> 1 movement (place a pillar)
  - Seat occupied in A, empty in B      -> 1 movement (remove a pillar)
  - Seat occupied in both, same color   -> 0 movements (leave as-is)
  - Seat occupied in both, diff color   -> 1 movement (swap the pillar)

This models the real physical effort: every seat that needs a pillar
added, removed, or color-swapped costs exactly one "touch" by the
team between rounds. The total cost from A -> B is the sum over all
21 seats.

We then solve for the ordering (a Hamiltonian path, not a cycle -
since this is a one-time test session, not a loop back to start)
that minimizes total movement cost across all consecutive transitions.
For n=7 this is solved by brute-force over all permutations (7! = 5040,
trivial to compute exactly), fixing nothing in particular - all
permutations are considered so the true global optimum is found.
"""

import os
import sys
from itertools import permutations

# Re-use the input parser + config file format from the main script
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from generate_field_from_pillars import parse_config_file, parse_pillar_input


def config_to_seat_map(pillar_string):
    """Converts a pillar string like '5G 6G 10G' into a dict
    {seat_number: color} for seats 1-21 (missing seats = empty)."""
    pairs = parse_pillar_input(pillar_string)
    return dict(pairs)  # {number: 'R'/'G'}


def movement_cost(seat_map_a, seat_map_b):
    """Computes the number of pillar movements needed to go from the
    field state in seat_map_a to the field state in seat_map_b,
    across all 21 seats."""
    cost = 0
    all_seats = range(1, 22)
    for seat in all_seats:
        color_a = seat_map_a.get(seat)
        color_b = seat_map_b.get(seat)
        if color_a == color_b:
            continue  # both empty, or both occupied with same color -> no movement
        cost += 1  # place, remove, or color-swap -> 1 movement
    return cost


def compute_cost_matrix(names, seat_maps):
    """Returns a dict {(name_a, name_b): cost} for every ordered pair."""
    matrix = {}
    for a in names:
        for b in names:
            if a == b:
                continue
            matrix[(a, b)] = movement_cost(seat_maps[a], seat_maps[b])
    return matrix


def find_optimal_sequence(names, cost_matrix):
    """
    Brute-force search over all permutations of `names` to find the
    ordering (as a path, not a cycle) that minimizes the sum of
    consecutive transition costs. Exact for small n (n=7 -> 5040
    permutations, negligible runtime).

    Returns: (best_order, best_total_cost, all_results_sorted)
    """
    best_order = None
    best_total = None
    all_results = []

    for perm in permutations(names):
        total = 0
        for i in range(len(perm) - 1):
            total += cost_matrix[(perm[i], perm[i + 1])]
        all_results.append((perm, total))
        if best_total is None or total < best_total:
            best_total = total
            best_order = perm

    all_results.sort(key=lambda x: x[1])
    return best_order, best_total, all_results


def describe_transition(name_a, name_b, seat_map_a, seat_map_b):
    """Returns a human-readable description of what changes between
    two configs: pillars added, removed, and color-swapped."""
    added, removed, swapped, unchanged = [], [], [], []
    for seat in range(1, 22):
        ca = seat_map_a.get(seat)
        cb = seat_map_b.get(seat)
        if ca == cb:
            if ca is not None:
                unchanged.append(seat)
            continue
        if ca is None and cb is not None:
            added.append((seat, cb))
        elif ca is not None and cb is None:
            removed.append((seat, ca))
        else:
            swapped.append((seat, ca, cb))
    return added, removed, swapped, unchanged


def main():
    config_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "input_config", "pillar_configs_input.txt")
    configs = parse_config_file(config_file)  # list of (name, pillar_string)
    names = [name for name, _ in configs]
    seat_maps = {name: config_to_seat_map(pillar_string) for name, pillar_string in configs}

    print("=" * 70)
    print("Loaded configurations:")
    for name, pillar_string in configs:
        print(f"  {name}: {pillar_string}  ({len(seat_maps[name])} pillars)")
    print("=" * 70)

    cost_matrix = compute_cost_matrix(names, seat_maps)

    print("\nPairwise movement-cost matrix:")
    header = "      " + "  ".join(f"{n:>4}" for n in names)
    print(header)
    for a in names:
        row = [a.rjust(4)]
        for b in names:
            if a == b:
                row.append("   -")
            else:
                row.append(f"{cost_matrix[(a, b)]:>4}")
        print("  " + "  ".join(row))

    best_order, best_total, all_results = find_optimal_sequence(names, cost_matrix)

    print("\n" + "=" * 70)
    print(f"OPTIMAL SEQUENCE (minimizes total pillar movements across all transitions):")
    print("  " + " -> ".join(best_order))
    print(f"  Total movements across sequence: {best_total}")
    print("=" * 70)

    print("\nTransition-by-transition breakdown:")
    for i in range(len(best_order) - 1):
        a, b = best_order[i], best_order[i + 1]
        cost = cost_matrix[(a, b)]
        added, removed, swapped, unchanged = describe_transition(a, b, seat_maps[a], seat_maps[b])
        print(f"\n  {a} -> {b}   (cost = {cost} movements)")
        if added:
            print(f"    + Place:  " + ", ".join(f"seat {s} -> {c}" for s, c in added))
        if removed:
            print(f"    - Remove: " + ", ".join(f"seat {s} (was {c})" for s, c in removed))
        if swapped:
            print(f"    ~ Swap:   " + ", ".join(f"seat {s} ({c1}->{c2})" for s, c1, c2 in swapped))
        if unchanged:
            print(f"    = Keep:   seats {', '.join(str(s) for s in unchanged)} (unchanged)")

    print("\n" + "=" * 70)
    print("Top 5 candidate orderings (for comparison):")
    for perm, total in all_results[:5]:
        print(f"  {total:>3} movements:  " + " -> ".join(perm))

    return best_order, best_total, cost_matrix, seat_maps


if __name__ == "__main__":
    main()
