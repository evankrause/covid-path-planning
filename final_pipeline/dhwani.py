import pandas as pd
import math

OUTPUT_CSV = '../output/tenbenham_waiting_times.csv'
#
# print(coords)
#
# # Getting the minimum distance
# min_dist, itinerary = tsp(coords)
# print(min_dist)
# print(itinerary)

# itinerary = [0, 4, 3, 2, 7, 8, 9, 11, 12, 15, 22, 26, 25, 24, 27, 32, 33, 37, 40, 48, 44, 41, 36, 38, 42, 45, 46, 47, 43, 39, 35, 34, 31, 30, 29, 28, 23, 20, 21, 19, 18, 17, 16, 13, 14, 10, 6, 5, 1]

# get_closest gets the closest point in the room, based on your current point.

def get_closest(node, coords, visited, l):
    point = coords[node]
    min_dist = float('inf')
    min_point = 0
    for i in range(l):
        # print(i)
        if i != node and i not in visited:
            p = coords[i]
            dist = ((p[0]-point[0])**2 + (p[1]-point[1])**2)
            if dist < min_dist:
                min_dist = dist
                min_point = i

    return min_point

# nearest_neighbor is a simple, greedy algorithm which creates a tour of all the points by always hopping to the nearest neighbor.

def nearest_neighbor(start, coords, l):
    tour = [start]
    visited = set()
    visited.add(start)
    # print(len(coords))
    while len(visited) < l:
        # print(len(visited))
        node = get_closest(start, coords, visited, l)
        # print(node)
        visited.add(node)
        tour.append(node)
        start = node

    return tour

# The following function calculates the cost of the tour (total distance travelled).

def cost(tour, coords):
    c = 0
    for i in range(len(tour)-1):
        p1 = coords[tour[i]]
        p2 = coords[tour[i+1]]
        c += math.sqrt(((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2))
    return c

#2 opt. This algorithm takes the tour created by the neerest neighbor algorithm and tries to improve the cost by swiching edge pairs.

def two_opt(tour, coords, l):
    # improved = True
    # while improved:
    best = cost(tour, coords)
    #improved = False
    for i in range(l-3):
        for j in range(i+1, l-1):
            new1 = [tour[i], tour[j]]
            new2 = [tour[i+1], tour[j+1]]
            old1 = [tour[i], tour[i+1]]
            old2 = [tour[j], tour[j+1]]
            # print(i, j)
            gain = cost(old1, coords) + cost(old2, coords) - cost(new1, coords) - cost(new2, coords)
            if gain > 0:
                best -= gain
                tour = tour[:i+1] + tour[i+1:j+1][::-1] + tour[j+1:]
                #improved = True
                break

    return tour

# The following function calculates distances and angles between the points in the tour.
def distance_angles (tour, coords, l):

    dists = []
    angles = []

    for i in range(l-1):
        src = coords[tour[i]]
        dest = coords[tour[i+1]]
        d = math.sqrt((dest[0] - src[0])**2 + (dest[1]-src[1])**2)
        dists.append(d)

        angle = math.atan2(dest[1]-src[1], dest[0]-src[0])
        angles.append(angle)

    return dists, angles