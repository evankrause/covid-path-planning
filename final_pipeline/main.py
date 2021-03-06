# Main script to solve the UV Light optimization problem

import pandas as pd
from room import Room
from polygon_extraction import extract_polygon, construct_isValidLocation_function
from lp_solver import solve_full_lp, visualize_times, solve_naive, visualize_path
from order_path import *

######################
###   Parameters   ###
######################

# I/O Files
INPUT_FILE = '../floor_plans/hrilab_sledbot_twolasers3.pgm'
INPUT_YAML = '../floor_plans/hrilab_sledbot_twolasers3.yaml'
# INPUT_FILE = '../floor_plans/gmapping_tenbenham.pgm'
# INPUT_YAML = '../floor_plans/gmapping_tenbenham.yaml'
OUTPUT_CSV = '../output/waiting_times.csv'

# Robot parameters
ROBOT_HEIGHT = 1.2192  # Height of UV light, in meters
ROBOT_RADIUS = 0.4  # Distance from robot center to farthest point, in meters
ROBOT_WATTAGE = 55  # Power of the UV light in Watts (ie. J/sec)

# Global parameters
DISINFECTION_THRESHOLD = 1206  # Joules/meter^2

# Algorithm parameters. See documentation for the different variations
naive_solution = False
use_strong_visibility = True
use_strong_distances = True
scaling_method = 'none'  # must be in {'epsilon', 'branch_and_bound', 'none'}
ORTHOGONAL_TOL = 20  # Tolerance for orthogonal simplification, in pixels
ROBOT_EPSILON = 0.4  # Size of grid for discretization of possible robot
#   locations, in meters
ROOM_EPSILON = 0.4  # Size of grid for discretization of locations to
#   disinfect, in meters
# Smaller epsilon values guarantee that we find a
#   solution closer to optimal, assuming infinite speed
#   The right value should be determined experimentally


############################
###   Compute Solution   ###
############################

# Step 1: read input file (pixel-like image) and transform it to a simple polygon (with clearly marked in/out)
print('Extracting polygon')
polygon, gray_img, xy_to_pixel, meters_per_pixel = extract_polygon(INPUT_FILE, INPUT_YAML,
                                                                   ortho_tolerance=ORTHOGONAL_TOL)

is_valid_location = construct_isValidLocation_function(gray_img, xy_to_pixel, ROBOT_RADIUS, meters_per_pixel)

# Step 2: a Room object contains not only the boundary, but creates a discretized list of places
#         for the robot to guard (and list of places where the robot can actually move to)
print('Creating room')
room = Room(polygon, gray_img, xy_to_pixel, robot_buffer_meters=ROBOT_RADIUS, is_valid_guard=is_valid_location,
            room_eps=ROOM_EPSILON, guard_eps=ROBOT_EPSILON)

if naive_solution:
    solve_naive(room, ROBOT_HEIGHT, DISINFECTION_THRESHOLD)
else:
    # Step 3: we generate the LP problem and solve it.
    print('Solving lp')
    time, waiting_times, intensities = solve_full_lp(room, ROBOT_HEIGHT, ROBOT_RADIUS, ROBOT_WATTAGE,
                                                     use_strong_visibility, use_strong_distances, scaling_method,
                                                     DISINFECTION_THRESHOLD)

    # Step 4: Output a solution
    print('Total solution time:', time)
    print(room.guard_grid.shape)
    print(intensities.shape)

    # combine positions and waiting time
    locs_times = []
    for (x, y), t in zip(room.guard_grid, waiting_times):
        # We drop points that you stop less than a milisecond. HARDCODED
        if t > 1e-3:
            locs_times.append([x, y, t])

    # Graphical visualizations of the solution
    print('Visualizing solution')
    visualize_times(room, waiting_times)

    # find an ordered path
    tour = order_path(gray_img, locs_times, xy_to_pixel)

    # order locations using the calculated path
    ordered_locs_times = [locs_times[i] for i in tour]

    # display solution
    visualize_path(room, ordered_locs_times)

    # add orientation to ordered locations
    orientations = calculate_orientations(ordered_locs_times)

    # write solution to file
    rows = []
    for (x, y, t), o in zip(ordered_locs_times, orientations):
        rows.append({'x': x, 'y': y, 'orient': o, 'time': t})

    pd.DataFrame(rows).to_csv(OUTPUT_CSV, index=False)
