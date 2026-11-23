"""
A* Pathfinding - Weighted Graph Proof of Concept
------------------------------------------------
Run:
    python astar_tests.py

The program includes:
    1. A* implementation using a priority queue.
    2. Three weighted-graph test cases.
    3. A test for an unreachable goal.
    4. Expected-result checks.

Graph representation:
    graph = {
        "A": {"B": 2, "D": 4},
        "B": {"C": 1},
        ...
    }

Each edge weight is the cost of moving from one node to another.
This implementation supports directed graphs. For an undirected graph,
add the reverse edge explicitly.
"""

import heapq
from itertools import count
from math import inf


def astar(graph, start, goal, heuristic):
    """
    Find a minimum-cost path from start to goal using A*.

    Args:
        graph: Adjacency dictionary. Each node maps to {neighbor: edge_cost}.
        start: Starting node.
        goal: Destination node.
        heuristic: Dictionary mapping nodes to estimated remaining costs.

    Returns:
        (path, total_cost)
        If no path exists, returns (None, inf).

    Optimality:
        With nonnegative edge costs and an admissible heuristic
        (which estimates remaining cost without overestimating), this
        implementation returns an optimal path. It reopens nodes when
        a cheaper route is found, so the heuristic need not be consistent.
    """

    # Tie-breaker prevents Python from comparing node objects when
    # two entries have the same priority.
    counter = count()

    # Priority queue entries: (f_score, tie_breaker, g_score, node)
    open_set = [
        (heuristic.get(start, 0), next(counter), 0, start)
    ]

    # Cheapest known cost from start to each node.
    g_score = {start: 0}

    # Previous node on the cheapest known route.
    came_from = {}

    while open_set:
        _, _, queued_g, current = heapq.heappop(open_set)

        # Skip queue entries that are no longer the best known route.
        if queued_g != g_score.get(current):
            continue

        # Reconstruct the path once the goal is reached.
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)

            path.reverse()
            return path, queued_g

        # Explore each neighbor.
        for neighbor, edge_cost in graph.get(current, {}).items():
            if edge_cost < 0:
                raise ValueError("A* requires nonnegative edge costs.")

            tentative_g = queued_g + edge_cost

            if tentative_g < g_score.get(neighbor, inf):
                g_score[neighbor] = tentative_g
                came_from[neighbor] = current

                f_score = tentative_g + heuristic.get(neighbor, 0)

                heapq.heappush(
                    open_set,
                    (f_score, next(counter), tentative_g, neighbor)
                )

    # The goal cannot be reached from the start.
    return None, inf


def run_test(name, graph, heuristic, start, goal,
             expected_path, expected_cost):
    """Run one test and compare the result with the expected answer."""

    path, cost = astar(graph, start, goal, heuristic)

    passed = (path == expected_path and cost == expected_cost)
    status = "PASS" if passed else "FAIL"

    print(f"--- {name}: {status} ---")
    print(f"Start:          {start}")
    print(f"Goal:           {goal}")
    print(f"Optimal path:   {path}")
    print(f"Total cost:     {cost}")
    print(f"Expected path:  {expected_path}")
    print(f"Expected cost:  {expected_cost}")
    print()

    return passed


def main():

    # ==============================================================
    # TEST CASE 1: Basic weighted graph
    # Several routes exist. The cheapest route costs 6.
    # ==============================================================

    graph1 = {
        "A": {"B": 2, "D": 4},
        "B": {"C": 1, "D": 3},
        "C": {"G": 8},
        "D": {"E": 1},
        "E": {"G": 1},
        "G": {}
    }

    # Zero heuristic makes this test equivalent to Dijkstra's algorithm.
    heuristic1 = {
        "A": 0,
        "B": 0,
        "C": 0,
        "D": 0,
        "E": 0,
        "G": 0
    }

    # ==============================================================
    # TEST CASE 2: Multiple competing routes
    # Tests whether the algorithm selects the lowest total cost,
    # even when that route has more edges.
    # ==============================================================

    graph2 = {
        "A": {"B": 2, "C": 5, "D": 1},
        "B": {"E": 7},
        "C": {"E": 2},
        "D": {"C": 1, "F": 4},
        "E": {"G": 2},
        "F": {"G": 2},
        "G": {}
    }

    # These estimates do not overestimate the actual remaining costs.
    heuristic2 = {
        "A": 4,
        "B": 3,
        "C": 2,
        "D": 3,
        "E": 1,
        "F": 2,
        "G": 0
    }

    # ==============================================================
    # TEST CASE 3A: Blocked connection / alternative route
    # There is no A-to-B connection, so the algorithm must use D.
    # ==============================================================

    graph3a = {
        "A": {"D": 1},
        "B": {"C": 3},
        "C": {"G": 2},
        "D": {"E": 2},
        "E": {"G": 2},
        "G": {}
    }

    heuristic3a = {
        "A": 4,
        "B": 3,
        "C": 2,
        "D": 3,
        "E": 2,
        "G": 0
    }

    # ==============================================================
    # TEST CASE 3B: Unreachable goal
    # No sequence of edges connects A to G.
    # ==============================================================

    graph3b = {
        "A": {"B": 2},
        "B": {"C": 3},
        "C": {},
        "G": {}
    }

    heuristic3b = {
        "A": 0,
        "B": 0,
        "C": 0,
        "G": 0
    }

    # Run all tests.
    results = [
        run_test(
            "TEST CASE 1 - Basic weighted graph",
            graph1, heuristic1, "A", "G",
            ["A", "D", "E", "G"], 6
        ),
        run_test(
            "TEST CASE 2 - Competing routes",
            graph2, heuristic2, "A", "G",
            ["A", "D", "C", "E", "G"], 6
        ),
        run_test(
            "TEST CASE 3A - Alternative route around blocked connection",
            graph3a, heuristic3a, "A", "G",
            ["A", "D", "E", "G"], 5
        ),
        run_test(
            "TEST CASE 3B - Unreachable goal",
            graph3b, heuristic3b, "A", "G",
            None, inf
        )
    ]

    print("=" * 55)
    print(f"TEST SUMMARY: {sum(results)}/{len(results)} tests passed")
    print("=" * 55)


if __name__ == "__main__":
    main()
