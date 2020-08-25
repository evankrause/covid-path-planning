from __future__ import print_function
import math
import sys

from dijkstar import Graph, find_path
from pandas import np
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp


# TODO: optimize this method to get rid of the terribly slow image iterations
def order_path(gray_img, locs, xy_to_pixel):
    # starting location
    # TODO: pass this in dynamically
    start_loc = locs[0]

    # construct 2d cost map where each entry is cost to move from neighboring pixel
    h, w = np.shape(gray_img)

    # iterate over the entire image and build graph for dijkstra calculations
    print('Converting map into graph.')
    graph = Graph()
    # NOTE: this isn't using the values in the map yaml (0.196) for free_thresh but instead assumes
    # 0 = occupied,  205 = unknown, 254 = free
    free_thresh = 220
    diag_cost = np.sqrt(2)
    lin_cost = 1
    for py in range(1, h - 1):
        for px in range(1, w - 1):
            if gray_img[py][px] < free_thresh:
                # obstacle -- skip it
                continue
            i = py * w + px
            for py2 in range(py - 1, py + 2):
                for px2 in range(px - 1, px + 2):
                    if gray_img[py2][px2] < free_thresh:
                        # obstacle -- skip it
                        continue
                    if px == px2 and py == py2:
                        # same as "center" pixel -- skip it
                        continue

                    i2 = py2 * w + px2
                    if px == px2 or py == py2:
                        # straight up or down pixel
                        graph.add_edge(i, i2, lin_cost)
                    else:
                        # diag pixel
                        graph.add_edge(i, i2, diag_cost)

    # perform dijkstra cost calculation for each disinfection location pair
    # to build cost array for traveling salesman problem
    print('Building cost matrix for TSP problem.')
    num_locs = len(locs)
    costs = [[0] * num_locs for i in range(num_locs)]
    for loc_i in range(num_locs):
        costs[loc_i][loc_i] = 0
        (px, py) = xy_to_pixel(locs[loc_i][0], locs[loc_i][1])
        src_i = py * w + px
        for loc_j in range(loc_i + 1, num_locs):
            (px, py) = xy_to_pixel(locs[loc_j][0], locs[loc_j][1])
            dst_i = py * w + px
            path_info = find_path(graph, src_i, dst_i)
            cost = int(round(path_info.total_cost))
            costs[loc_i][loc_j] = cost
            costs[loc_j][loc_i] = cost

    # find disinfection location closest to and furthest from starting location (i.e., where robot currently is)
    # this will serve as the start and end locations for the tsp problem
    print('Finding starting and ending disinfection locations.')
    (px, py) = xy_to_pixel(start_loc[0], start_loc[1])
    src_i = py * w + px
    start_i = -1
    end_i = -1
    min_cost = sys.maxsize
    max_cost = 0
    for loc_i in range(num_locs):
        (px, py) = xy_to_pixel(locs[loc_i][0], locs[loc_i][1])
        dst_i = py * w + px
        path_info = find_path(graph, src_i, dst_i)
        cost = int(round(path_info.total_cost))
        if cost < min_cost:
            min_cost = cost
            start_i = loc_i
        if cost > max_cost:
            max_cost = cost
            end_i = loc_i

    # feed cost array into tsp algorithm
    print('Solving TSP problem.')
    data = create_data_model(costs, start_i, end_i)
    tour = solve_tsp(data)

    return tour


def create_data_model(costs_matrix, start_index, end_index):
    """Stores the data for the problem."""
    data = {'distance_matrix': costs_matrix, 'num_vehicles': 1, 'starts': [start_index], 'ends': [end_index]}
    return data


def get_solution(manager, routing, solution):
    """Prints solution on console."""
    print('Objective: {} miles'.format(solution.ObjectiveValue()))
    index = routing.Start(0)
    plan_output = 'Route for vehicle 0:\n'
    route_distance = 0
    tour = []
    while not routing.IsEnd(index):
        tour.append(manager.IndexToNode(index))
        plan_output += ' {} ->'.format(manager.IndexToNode(index))
        previous_index = index
        index = solution.Value(routing.NextVar(index))
        route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
    plan_output += ' {}\n'.format(manager.IndexToNode(index))
    print(plan_output)
    plan_output += 'Route distance: {}miles\n'.format(route_distance)

    return tour


def solve_tsp(data):
    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                           data['num_vehicles'], data['starts'], data['ends'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Get solution as list of ordered indices
    if solution:
        return get_solution(manager, routing, solution)
    else:
        return []


# calculates angles between the points in the tour.
def calculate_orientations(locations):
    orientations = [0]
    for i in range(len(locations) - 1):
        src = locations[i]
        dest = locations[i + 1]
        angle = math.atan2(dest[1] - src[1], dest[0] - src[0])
        orientations.append(angle)

    return orientations
