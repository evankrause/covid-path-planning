# Main script to solve the UV Light optimization problem

import pandas as pd
from room import Room
from polygon_extraction import extract_polygon
from lp_solver import solve_full_lp, visualize_times
import math
from tsp import tsp
from dhwani import *

######################
###   Parameters   ###
######################

# I/O Files
INPUT_FILE = '../floor_plans/gmapping_tenbenham_cropped.png'
OUTPUT_FILE = '../output/tenbenham_polygon.png'
OUTPUT_CSV = '../output/tenbenham_waiting_times.csv'

# Robot dimensions
METERS_PER_PIXEL = 0.05 # Constant to determine the correspondence between input pixels and dimensions in real space
                        # HARDCODED. Should read from input file instead
ROBOT_HEIGHT = 1.2192   # in meters (currently set to 4 feet)
EPSILON = 0.5           # Tolerance error. Units of ROBOT_HEIGHT meters.
                        #   A smaller epsilon guarantees that we find a
                        #   solution closer to optimal, assuming infinite speed
                        #   The right value should be determined experimentally
MIN_INTENSITY = 1       # Threshold of energy each point in the room must
                        #   receive, assuming that the UV light emits
                        #   1/distance_in_pixels^2 units of energy

# Algorithm parameters. See documentation for the different variations
use_strong_visibility = False
use_strong_distances = False
scaling_method = 'epsilon' # must be in {'epsilon', 'branch_and_bound', 'none'}


############################
###   Compute Solution   ###
############################

units_per_pixel =  METERS_PER_PIXEL * 1/ROBOT_HEIGHT
robot_height_pixels = ROBOT_HEIGHT * 1/METERS_PER_PIXEL
print('Scaled robot height', robot_height_pixels)

# Step 1: read input file (pixel-like image) and transform it to a simple polygon (with clearly marked in/out)
print('Extracting polygon')
polygon = extract_polygon(INPUT_FILE, OUTPUT_FILE)

# Step 2: a Room object contains not only the boundary, but creates a discretized list of places
#         for the robot to guard (and list of places where the robot can actually move to)
print('Creating room')
room = Room(polygon, units_per_pixel, room_eps = EPSILON, guard_eps = EPSILON, guard_scale = 1)

# Step 3: we generate the LP problem and solve it.
print('Solving lp')
time, waiting_times, intensities = solve_full_lp(room, robot_height_pixels, use_strong_visibility, use_strong_distances, scaling_method, MIN_INTENSITY)

# Step 4: Output a solution
print('Total solution time:', time)
print(room.guard_grid.shape)
print(intensities.shape)

# Create a csv of all positions and waiting time
rows = []
for (x, y), t in zip(room.guard_grid, waiting_times):
    # We drop points that you stop less than a milisecond. HARDCODED
    if t > 1e-3:
        rows.append({'x': x, 'y': y, 'time': t})
pd.DataFrame(rows).to_csv(OUTPUT_CSV, index=False)

# Graphical visualizations of the solution
print('Visualizing solution')
visualize_times(room, waiting_times)

# all the following code is new
# READING THE CSV.
# coords contains the (x,y) co-ordinates of the points in the room.
# We need to go to each and every point in the coords list.

coords_pdf = pd.read_csv(OUTPUT_CSV)
coords = []
for i, row in coords_pdf.iterrows():
    x = row['x']
    y = row['y']
    coords.append((x, y))

l = len(coords)

tour = nearest_neighbor(0, coords, l)
print(tour)

for i in range(5):
    tour = two_opt(tour, coords, l)
    print(cost(tour, coords))

print(tour)

dists, angles = distance_angles(tour, coords, l)
angles += [0]
#
coords_pdf['orient'] = angles
coords_pdf.to_csv(OUTPUT_CSV)
#
