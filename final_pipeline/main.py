# Main script to solve the UV Light optimization problem

import pandas as pd
from room import Room
from polygon_extraction import extract_polygon, construct_isValidLocation_function
from lp_solver import solve_full_lp, visualize_times, solve_naive, visualize_our_path
from shapely.geometry import box
import matplotlib.pyplot as plt
from dhwani import *

######################
###   Parameters   ###
######################

# I/O Files
INPUT_FILE = '../floor_plans/hrilab_sledbot_twolasers3.pgm'
INPUT_YAML = '../floor_plans/hrilab_sledbot_twolasers3.yaml'
OUTPUT_CSV = '../output/tenbenham_waiting_times.csv'

# Robot dimensions
ROBOT_HEIGHT = 1.2192   # in meters (currently set to 4 feet)
ROBOT_BUFFER = 0.4      # "Radius" of the robot, in meters
                        #   The center of the robot will stay at least ROBOT_BUFFER meters from walls
ORTHOGONAL_TOL = 20     # Tolerance for orthogonal simplification, in pixels
EPSILON = 0.4           # Size of grid for discretization. Units of meters.
                        #   A smaller epsilon guarantees that we find a
                        #   solution closer to optimal, assuming infinite speed
                        #   The right value should be determined experimentally
MIN_INTENSITY = 1       # Threshold of energy each point in the room must
                        #   receive, assuming that the UV light emits
                        #   1/distance_in_pixels^2 units of energy

# Algorithm parameters. See documentation for the different variations
naive_solution = False
use_strong_visibility = True
use_strong_distances = True
scaling_method = 'none' # must be in {'epsilon', 'branch_and_bound', 'none'}

############################
###   Compute Solution   ###
############################

# Step 1: read input file (pixel-like image) and transform it to a simple polygon (with clearly marked in/out)
print('Extracting polygon')
polygon, gray_img, xy_to_pixel, meters_per_pixel = extract_polygon(INPUT_FILE, INPUT_YAML, ortho_tolerance = ORTHOGONAL_TOL)

is_valid_location = construct_isValidLocation_function(gray_img, xy_to_pixel, ROBOT_BUFFER, meters_per_pixel)

# Step 2: a Room object contains not only the boundary, but creates a discretized list of places
#         for the robot to guard (and list of places where the robot can actually move to)
print('Creating room')
room = Room(polygon, robot_buffer_meters = ROBOT_BUFFER, is_valid_guard = is_valid_location, room_eps = EPSILON, guard_eps = EPSILON)

for guard_pt in room.guard_grid:
    plt.scatter(*guard_pt, color = 'blue')
plt.plot(*polygon.exterior.xy, color = 'red')
plt.show()

if naive_solution:
    solve_naive(room, ROBOT_HEIGHT, MIN_INTENSITY)
else:
    # Step 3: we generate the LP problem and solve it.
    print('Solving lp')
    time, waiting_times, intensities = solve_full_lp(room, ROBOT_HEIGHT, use_strong_visibility, use_strong_distances, scaling_method, MIN_INTENSITY)

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
    
    coords_pdf['orient'] = angles
    coords_pdf.to_csv(OUTPUT_CSV)
    
    visualize_our_path(room, waiting_times, coords_pdf, tour)



##################################
# rot_room = Room(new_pol, units_per_pixel, robot_buffer_pixels = robot_buffer_pixels, room_eps = EPSILON, guard_eps = EPSILON)
#
# ax = plt.axes()
# ax.axis('equal')
# ax.plot(*rot_room.room.exterior.xy)
# plt.show()

#tour_pts = map(lambda x: coords[x], tour)
#tour_pt_list = list(tour_pts)
#print(tour_pt_list)
#modified_tour = tour[:5] + [tour[6]] + [tour[5]] + tour[7:]
#print(modified_tour)

#small_room_lst = [(110.0, 84.0), (97.0, 86.0)]

#room_corner_lst = list(polygon.exterior.coords)
#print(room_corner_lst)

"""
new_coords, s2 = rotate_room((1, 2), (-1, -1), room_corner_lst)
print('Rotated successfully with no errors')
print(new_coords)
geometry_points = []
for pt in new_coords:
    p = Point(pt[0], pt[1])
    geometry_points.append(p)

new_pol = Polygon([[p.x, p.y] for p in geometry_points])
# print (polygon)
# print ('yes')
"""

# check if the tour hits any of the walls of the room.
#for i in range(len(tour_pt_list) - 1):
#    s1 = tour_pt_list[i]
#    s2 = tour_pt_list[i + 1]
#    print(s1, s2)
#    print(i, hits_wall(s1, s2, room_corner_lst)) #room_corner_lst

#print(hits_1_wall((108, 74), (95, 127), [(110, 84), (97,86)]))
##################################
