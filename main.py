import sys

import utility as utility
import loader as loader
import numpy as np


def main():

    # Paths to the data and solution files.
    vrp_file = "data/n32-k5.vrp"  # "data/n80-k10.vrp"
    sol_file = "data/n32-k5.sol"  # "data/n80-k10.sol"

    # Loading the VRP data file.
    px, py, demand, capacity, depot = loader.load_data(vrp_file)

    # Displaying to console the distance and visualizing the optimal VRP solution.
    vrp_best_sol = loader.load_solution(sol_file)
    best_distance = utility.calculate_total_distance(vrp_best_sol, px, py, depot)
    print("Best VRP Distance:", best_distance)
    utility.visualise_solution(vrp_best_sol, px, py, depot, "Optimal Solution")

    # Executing and visualizing the nearest neighbour VRP heuristic.
    # Uncomment it to do your assignment!

    nnh_solution = nearest_neighbour_heuristic(px, py, demand, capacity, depot)
    nnh_distance = utility.calculate_total_distance(nnh_solution, px, py, depot)
    print("Nearest Neighbour VRP Heuristic Distance:", nnh_distance)
    utility.visualise_solution(nnh_solution, px, py, depot, "Nearest Neighbour Heuristic")

    # Executing and visualizing the saving VRP heuristic.
    # Uncomment it to do your assignment!
    
    # sh_solution = savings_heuristic(px, py, demand, capacity, depot)
    # sh_distance = utility.calculate_total_distance(sh_solution, px, py, depot)
    # print("Saving VRP Heuristic Distance:", sh_distance)
    # utility.visualise_solution(sh_solution, px, py, depot, "Savings Heuristic")


def nearest_neighbour_heuristic(px, py, demand, capacity, depot):

    """
    Algorithm for the nearest neighbour heuristic to generate VRP solutions.

    :param px: List of X coordinates for each node.
    :param py: List of Y coordinates for each node.
    :param demand: List of each nodes demand.
    :param capacity: Vehicle carrying capacity.
    :param depot: Depot.
    :return: List of vehicle routes (tours).
    """
    # Initialise lists
    routes = list()
    tour = list()
    visited = list()
    tour_demand = 0.
    tour.append(depot)
    # Until all nodes are visited
    while True:
        # Get feasible nodes in order of distance only from the depot
        if len(tour) == 1:
            tour, distances = get_nodes_from_depot(tour, visited, capacity, px, py, depot, tour_demand, demand)
        for i in range(len(px)):  # For the non-depot nodes
            # Get the distance of other nodes to this from smallest to largest
            distances[i] = utility.calculate_euclidean_distance(px, py, tour[-1], i)
            distances_ordered = dict(sorted(distances.items(), key=lambda item: item[1]))
            closest = list(distances_ordered.keys())

            for j in closest:
                if j not in tour and j not in visited:
                    if capacity - tour_demand >= 0:
                        # Add nodes to tour
                        tour.append(j)
                        visited.append(j)
                        tour_demand += demand[j]
                    else:
                        # Add tour to route
                        tour.append(depot)
                        routes.append(tour)
                        # Clear tour
                        tour_demand = 0.
                        tour.clear()
                        tour.append(depot)
        if len(visited) == len(px) - 1:
            break
    return routes


def get_nodes_from_depot(tour, visited, capacity, px, py, depot, tour_demand, demand):
    distances = {}
    for i in range(len(px)):  # For all nodes
        # Get the distance from the depot
        distances[i] = utility.calculate_euclidean_distance(px, py, depot, i)
    # Order the dictionary of euclidean distances from smallest to largest
    distances_ordered = dict(sorted(distances.items(), key=lambda item: item[1]))
    closest = list(distances_ordered.keys())

    for i in closest:  # For nodes ordered by distance
        if i not in tour and i not in visited:  # Check feasible
            if capacity - tour_demand >= 0:
                tour.append(i)  # Add node to tour
                visited.append(i)  # Mark as visited
                tour_demand += demand[i]  # Update the current demand of tour
            else:
                continue  # If not check next
            break
    return tour, distances


def savings_heuristic(px, py, demand, capacity, depot):

    """
    Algorithm for Implementing the savings heuristic to generate VRP solutions.

    :param px: List of X coordinates for each node.
    :param py: List of Y coordinates for each node.
    :param demand: List of each nodes demand.
    :param capacity: Vehicle carrying capacity.
    :param depot: Depot.
    :return: List of vehicle routes (tours).
    """

    # TODO - Implement the Saving Heuristic to generate VRP solutions.
    # 1. Initialise routes (1,vi,1) for each node vi (except depot)

    # 2. Compute and store the savings for each possible merge
    #       - saving(vi,vj) = L(vi,1) + L(1, vj) - L(vi,vj)

    # 3. Check all possible/feasible route merges
    #       - merge route1 and route2: merge the last non-depot node of route1 and
    #          the first non-depot node of route2

    # 4. Select the merge with the largest saving and merge the routes

    # 5. Repeat 3 and 4 until no more merges can be done

    # Initialise routes (skip the first - depot)
    routes = []
    for i in range(len(px))[1:]:
        routes.append([depot, i, depot])
    # Compute savings - record in dictionary
    savings = {}
    for i in range(len(px)):
        for j in range(len(px)):
            if i is j:
                break
            savings[str(i) + str(j)] = saving(i, j, depot, px, py)

    # Check all feasible/route merges
    merges = {}
    for i in range(len(routes)):
        for j in range(len(routes)):
            if i is j:
                break
            if merge_feasible(i, j, routes, demand, capacity):
                # merge = [i, j]
                merges[str(i) + str(j)] = [i, j]

    # Merge highest saving merge
    while True:
        merge_key = None
        highest_saving = None
        for current_merge_key in merges:
            current_merge_saving = savings[str(merges[current_merge_key][0]), str(merges[current_merge_key][1])]
            if current_merge_saving > highest_saving:
                merge_key = current_merge_key
                highest_saving = current_merge_saving
        # If there is no possible merge
        if merge_key is None:
            break
        else:
            do_merge(merge_key, merges, routes)

    return routes


def saving(node1, node2, depot, px, py):
    return utility.calculate_euclidean_distance(px, py, node1, depot) + \
           utility.calculate_euclidean_distance(px, py, depot, node2) - \
           utility.calculate_euclidean_distance(px, py, node1, node2)


def route_wo_depot(route, depot):
    new_route = []
    for i in route:
        if i is not depot:
            new_route.append(i)
    return new_route


def merge_feasible(i, j, routes, demand, capacity):
    # Check demand, capacity, and a node doesn't exist in both loops (possible?)
    route1 = routes[i]
    route2 = routes[j]
    route1.remove(route1[-1])
    route2.remove(route2[0])
    new_route = list(route1.extend(route2))
    current_demand = 0
    for k in new_route:
        current_demand += demand[k]
    if current_demand > capacity:
        return False
    else:
        return True


def do_merge(merge_key, merges, routes):
    # Get routes
    route1 = routes[merges[merge_key][0]]
    route2 = routes[merges[merge_key][1]]
    # Do merge
    route1.remove(route1[-1])
    route2.remove[0]
    new_route = route1.extend(route2)
    # Insert new route and remove old routes
    routes[route1] = new_route
    routes.remove(route2)

if __name__ == '__main__':
    main()
