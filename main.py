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
    tourDemand = 0
    while True: #Continues until all nodes are in the route
        nextNode = None
        nextNodeDist = sys.maxsize
        #Find nearest feasible node
        for node in range(len(px)):
            # If the node is unvisited
            if node not in tour and not routeContainsNode(route, node):
                # If capacity not exceeded after insertion
                if tourDemand + demand[node] <= capacity:
                    # See if nearest so far
                    if len(tour) == 0:
                        nextNode = node
                        nextNodeDist = utility.calculate_euclidean_distance(px,py, depot, node)
                    elif utility.calculate_euclidean_distance(px, py, tour[-1], node) < nextNodeDist:
                        nextNode = node

        # If no feasible node
        if nextNode is None:
            route.append(tour)
            tour.clear()
        else:
            tour.append(nextNode)
        if routeCheck(route, len(px)):
            break
    return route


def routeCheck(route, nodesLength):
    count: int = 0
    for tour in route:
        for i in tour:
            count += 1
    return count == nodesLength

def routeContainsNode(route, node):
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

    return None


if __name__ == '__main__':
    main()
