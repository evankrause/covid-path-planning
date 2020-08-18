import pandas as pd
import math
import numpy as np


# # Getting the minimum distance
# min_dist, itinerary = tsp(coords)
# print(min_dist)
# print(itinerary)


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

# s1, s2 defines a line segment. We are going to translate/rotate all points so that s1 becomes the origin and
# s2 is on the x axis. All the vertices of the room also undergo the same transformation. So, all relative positions are prserved.

def rotate_room(s1, s2, room):
    x1 = s1[0]
    y1 = s1[1]

    x2 = s2[0]
    y2 = s2[1]

    # translates s1 to the origin
    trans = (-x1, -y1)
    # at this point s2 will be (x2 - x1, y2 - y1)
    #rot_angle = -1 * math.atan2((y2-y1), (x2-x1))
    #c, s = np.cos(rot_angle), np.sin(rot_angle)

    y_travel = y2 - y1 # to get the -ve angle.
    x_travel = x2 - x1
    hyp = math.sqrt(y_travel**2 + x_travel**2)

    c, s = x_travel/hyp, y_travel/hyp

    rot_mx = np.array(((c, -s), (s, c)))

    rot_room = []

    for i in range(len(room)):
        point = room[i]
        n_point = [None, None]
        n_point[0] = point[0] + trans[0]
        n_point[1] = point[1] + trans[1]

        xx = n_point[0] * c + n_point[1] * s # could be done with np.array, matrix multiplication.
        yy = -n_point[0] * s + n_point[1] * c
        rot_room += [(xx, yy)]

    x2 -= x1
    y2 -= y1
    xx = x2*c + y2*s
    s2 = (xx, 0)

    return rot_room, s2


# finds possible walls that path from s1 to s2 can cross
def hits_wall(s1, s2, room):
    rot_room, new_s2 = rotate_room(s1, s2, room)
    print (room, len(rot_room))

    it_did = False
    for i in range(len(rot_room)):
        pt1 = rot_room[i]
        idx = (i+1) % len(rot_room)
        pt2 = rot_room[idx]

        #print (pt1, pt2, new_s2)
        if pt1[1] * pt2[1] >= 0:
            it_did = it_did or False
        elif pt1[1] == pt2[1]:
            it_did = it_did or False
        else:
            x1 = pt1[0]
            y1 = pt1[1]
            x2 = pt2[0]
            y2 = pt2[1]
            x_cross = (y2*x1 - y1*x2)/(y2-y1)
            it_did = it_did or ((0 <= x_cross) and (x_cross <= new_s2[0]))

    return it_did

def hits_1_wall(s1, s2, sm_room):
    rot_room, new_s2 = rotate_room(s1, s2, sm_room)
    #print ('hits_1_wall')
    pt1 = rot_room[0]
    pt2 = rot_room[1]
    #print (sm_room, len(rot_room), pt1, pt2, new_s2)
    #print ('hits_1_wall')
    if pt1[1] * pt2[1] >= 0:
        return False
    elif pt1[1] == pt2[1]:
        return False
    else:
        x1 = pt1[0]
        y1 = pt1[1]
        x2 = pt2[0]
        y2 = pt2[1]
        x_cross = (y2 * x1 - y1 * x2) / (y2 - y1)
        #print (x_cross)
        return (0 <= x_cross) and (x_cross <= new_s2[0])




