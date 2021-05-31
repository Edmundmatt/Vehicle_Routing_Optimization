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
    
    sh_solution = savings_heuristic(px, py, demand, capacity, depot)
    sh_distance = utility.calculate_total_distance(sh_solution, px, py, depot)
    print("Saving VRP Heuristic Distance:", sh_distance)
    utility.visualise_solution(sh_solution, px, py, depot, "Savings Heuristic")


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

    # TODO - Implement the Nearest Neighbour Heuristic to generate VRP solutions.
    # 1. Initialise a solution: route starting from the depot

    # 2. Append the nearest feasible node to the end of the current route
    #       Feasible: node is unvisited, after insertion total demand of the route
    #       does not exceed the capacity

    # 3. If no feasible node, close the current route (return to the depot)
    #       Create a new route starting from the depot

    # 4. Repeat 2. and 3. until all nodes are visited
    # Nodes identified by index
    route = list()
    tour = list()
    tour_demand = 0
    while True: #Continues until all nodes are in the route
        next_node = None
        next_node_dist = sys.maxsize
        #Find nearest feasible node
        for node in range(len(px)):
            # If the node is unvisited
            if node not in tour and not route_contains_node(route, node):
                # If capacity not exceeded after insertion
                if tour_demand + demand[node] <= capacity:
                    # See if nearest so far
                    if len(tour) == 0:
                        next_node = node
                        next_node_dist = utility.calculate_euclidean_distance(px,py, depot, node)
                    elif utility.calculate_euclidean_distance(px, py, tour[-1], node) < next_node_dist:
                        next_node = node

        # If no feasible node
        if next_node is None:
            tour.insert(0, depot)
            tour.append(depot)
            route.append(tour)
            tour.clear()
            tour_demand = 0
        else:
            tour.append(next_node)
            tour_demand += demand[next_node]
        # Break if all nodes are in the route
        if route_check(route, len(px), depot):
            break
    return route


def route_check(route, nodes_length, depot):
    count: int = 0
    for tour in route:
        for i in tour:
            if i != depot:
                count += 1
    return count == nodes_length

def route_contains_node(route, node):
    for tour in route:
        for i in tour:
            if i is node:
                return True
    return False

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
