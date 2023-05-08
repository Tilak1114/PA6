import math

import cv2
import sys
import numpy as np

MAX_ITERS = 1000
INTERSECTION_THRESH = 2

map_file = sys.argv[1]
start_pos = (int(sys.argv[2]), int(sys.argv[3]))
goal_pos = (int(sys.argv[4]), int(sys.argv[5]))

map_file = map_file.strip()

if map_file != "map1" and map_file != "map2":
    raise ValueError("Invalid map file name")

gray_map = cv2.imread("maps/" + map_file + ".png", cv2.IMREAD_GRAYSCALE)

color_map = cv2.imread("maps/" + map_file + ".png")

gray_map = np.array(gray_map)

all_random_points = {start_pos: [(start_pos[0], start_pos[1], 0)]}

radius = 8

color = (0, 0, 255)
thickness = -1
cv2.circle(color_map, start_pos, radius, color, thickness)
cv2.circle(color_map, goal_pos, radius, color, thickness)

# Find obstacles
obstacles = []
for i in range(len(gray_map)):
    for j in range(len(gray_map[0])):
        if gray_map[i][j] == 0:
            obstacles.append((j, i))

prev_random_point = start_pos


def dijkstra(adj_list, start_node):
    dij_res = {}
    visited = {}

    # (cost, parentx, parenty)

    queue = [start_node]

    for k in adj_list.keys():
        cost = math.inf
        par_x = -1
        par_y = -1
        if k == start_node:
            cost = 0
            par_x = start_node[0]
            par_y = start_node[1]
        dij_res[k] = [cost, par_x, par_y]
        visited[k] = False

    while queue:
        curr_node = queue.pop(0)
        neighbors = adj_list[curr_node]

        visited[curr_node] = True

        for n in neighbors:
            if curr_node[0] == n[0] and curr_node[1] == n[1]:
                continue

            n_x, n_y, n_c = n

            parent_node_cost = dij_res[curr_node][0]
            current_neigh_cost = n_c

            if parent_node_cost + current_neigh_cost <= dij_res[(n_x, n_y)][0]:
                dij_res[(n_x, n_y)][0] = parent_node_cost + current_neigh_cost
                dij_res[(n_x, n_y)][1] = curr_node[0]
                dij_res[(n_x, n_y)][2] = curr_node[1]

            if not visited[(n_x, n_y)]:
                queue.append((n_x, n_y))
                visited[(n_x, n_y)] = True

    return dij_res


def does_point_interest_line(point, line_start, line_end):
    x1, y1 = line_start
    x2, y2 = line_end
    if x2 - x1 == 0:
        return True
    m = (y2 - y1) / (x2 - x1)
    b = y1 - m * x1

    expected_y = m * point[0] + b

    return abs(point[1] - expected_y) < INTERSECTION_THRESH


def is_point_on_line_segment(point, line_start, line_end):
    min_x = min(line_start[0], line_end[0])
    max_x = max(line_start[0], line_end[0])
    min_y = min(line_start[1], line_end[1])
    max_y = max(line_start[1], line_end[1])

    return min_x <= point[0] <= max_x and min_y <= point[1] <= max_y


def find_closest_point(point, points):
    closest_dist = math.inf
    closest_point = points[0]
    for p in points:
        current_dist = np.linalg.norm(np.array(point) - np.array(p))
        if current_dist < closest_dist:
            closest_dist = current_dist
            closest_point = p

    return closest_point, closest_dist


def map_comp_intersection_check(obstacle_list, map_with_line):
    for obstacle in obstacle_list:
        if map_with_line[tuple(reversed(obstacle))] == 255:
            return True

    return False


def geometry_based_intersection_check(
        point1,
        point2, obstacle_list):
    intersection = False
    for obstacle_point in obstacle_list:
        intersection = is_point_on_line_segment(
            obstacle_point,
            point1,
            point2
        ) and does_point_interest_line(
            obstacle_point,
            point1,
            point2
        )
        if intersection:
            break
    return intersection


point_radius = len(gray_map)

curr_iter = 0

while True:
    curr_iter += 1
    radius_x_start = max(0, prev_random_point[0] - point_radius)
    radius_x_end = min(len(gray_map[0]), prev_random_point[0] + point_radius)
    radius_y_start = max(0, prev_random_point[1] - point_radius)
    radius_y_end = min(len(gray_map), prev_random_point[1] + point_radius)

    rand_x = np.random.randint(radius_x_start, radius_x_end)
    rand_y = np.random.randint(radius_y_start, radius_y_end)

    # filter = np.zeros(gray_map.shape, dtype=np.uint8)
    #
    # thickness = 1
    # cv2.line(filter, prev_random_point, (rand_x, rand_y), 255.0, thickness)

    does_obs_intersect = geometry_based_intersection_check(
        prev_random_point, (rand_x, rand_y), obstacles
    )
    obs_idx = 0

    if not does_obs_intersect:
        closest_point, dist = find_closest_point((rand_x, rand_y), list(all_random_points.keys()))

        does_closest_point_intersect = geometry_based_intersection_check(
            closest_point, (rand_x, rand_y), obstacles
        )

        if does_closest_point_intersect:
            continue

        all_random_points[closest_point].append((rand_x, rand_y, dist))

        # draw red square for the random point
        size = 4

        rect_x_s = int(rand_x - size / 2)
        rect_y_s = int(rand_y - size / 2)

        rect_x_e = int(rand_x + size / 2)
        rect_y_e = int(rand_y + size / 2)

        color = (0, 0, 255)
        thickness = -1
        cv2.rectangle(color_map,
                      (rect_x_s, rect_y_s),
                      (rect_x_e, rect_y_e),
                      color,
                      thickness
                      )

        color = (0, 255, 0)
        thickness = 1
        cv2.line(color_map, closest_point, (rand_x, rand_y), color, thickness)

        # cv2.imshow('image', color_map)
        # cv2.waitKey(4)
        # cv2.destroyAllWindows()

        all_random_points[(rand_x, rand_y)] = [(rand_x, rand_y, 0)]
        prev_random_point = (rand_x, rand_y)

    if False and curr_iter == MAX_ITERS:
        print("Failed!")

        cv2.imshow('image', color_map)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        exit(0)

    prev_to_goal_intersection = geometry_based_intersection_check(
        prev_random_point,
        goal_pos,
        obstacles
    )

    if prev_to_goal_intersection:
        continue
    else:
        break

color = (0, 255, 0)
thickness = 1
cv2.line(color_map, prev_random_point, goal_pos, color, thickness)

goal_prev_dist = np.linalg.norm(np.array(goal_pos) - np.array(prev_random_point))
all_random_points[prev_random_point].append((goal_pos[0], goal_pos[1], goal_prev_dist))
all_random_points[goal_pos] = [(goal_pos[0], goal_pos[1], 0)]

dijkstra_result = dijkstra(all_random_points, start_pos)

# Backtrack
back_current = goal_pos
while back_current != start_pos:
    _, parent_x, parent_y = dijkstra_result[back_current]

    color = (255, 0, 0)
    thickness = 2
    cv2.line(color_map, back_current, (parent_x, parent_y), color, thickness)

    back_current = (parent_x, parent_y)

cv2.imshow('image', color_map)
cv2.waitKey(0)
cv2.destroyAllWindows()

cv2.imwrite(map_file+".png", color_map)
