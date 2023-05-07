import heapq
import sys
import numpy as np
import cv2
import math

MAX_ITERS = 1000

# map_file = sys.argv[1]
# start_pos = (int(sys.argv[2]), int(sys.argv[3]))
# goal_pos = (int(sys.argv[4]), int(sys.argv[5]))
#
# map_file = map_file.strip()
#
# if map_file != "map1" and map_file != "map2":
#     raise ValueError("Invalid map file name")

start_pos = (50, 50)
goal_pos = (500, 400)

gray_map = cv2.imread("maps/" + "map1" + ".png", cv2.IMREAD_GRAYSCALE)

color_map = cv2.imread("maps/" + "map1" + ".png")

gray_map = np.array(gray_map)

all_random_points = {start_pos: [(start_pos[0], start_pos[1], 0)]}

radius = 6

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

# obs_check = np.zeros((550, 550))
#
# for obs in obstacles:
#     obs_check[obs] = 255
#
# cv2.imshow('image', obs_check)
# cv2.waitKey(0)

prev_random_point = start_pos


def dijikstra(adj_list, start_node):
    dij_res = {}
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

    while queue:
        curr_node = queue.pop(0)
        neighbors = adj_list[curr_node]

        for n in neighbors:
            if curr_node[0] == n[0] and curr_node[1] == n[1]:
                continue

            n_x, n_y, n_c = n

            parent_node_cost = dij_res[curr_node][0]
            current_neigh_cost = n_c

            if parent_node_cost+current_neigh_cost <= dij_res[(n_x, n_y)][0]:
                dij_res[(n_x, n_y)][0] = parent_node_cost+current_neigh_cost
                dij_res[(n_x, n_y)][1] = curr_node[0]
                dij_res[(n_x, n_y)][2] = curr_node[1]

            queue.append((n_x, n_y))

    return dij_res


def does_point_interest_line(point, line_start, line_end):
    x1, y1 = line_start
    x2, y2 = line_end
    if x2 - x1 == 0:
        return True
    m = (y2 - y1) / (x2 - x1)
    b = y1 - m * x1

    expected_y = m * point[0] + b

    return abs(point[1] - expected_y) < 1


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


# point_radius = len(gray_map)
point_radius = 150

curr_iter = 0

# curr_iter < MAX_ITERS
while True:
    curr_iter += 1
    radius_x_start = max(0, prev_random_point[0] - point_radius)
    radius_x_end = min(len(gray_map[0]), prev_random_point[0] + point_radius)
    radius_y_start = max(0, prev_random_point[1] - point_radius)
    radius_y_end = min(len(gray_map), prev_random_point[1] + point_radius)

    rand_x = np.random.randint(radius_x_start, radius_x_end)
    rand_y = np.random.randint(radius_y_start, radius_y_end)

    does_obs_intersect = False
    obs_idx = 0
    for obstacle_point in obstacles:
        obs_idx += 1

        # cv2.circle(color_map, obstacle_point, 1, (0, 0, 255), -1)
        #
        # if obs_idx % 1000 == 0:
        #     cv2.imshow('image', color_map)
        #     cv2.waitKey(1)
        #     cv2.destroyAllWindows()

        prev_y, prev_x = prev_random_point

        # color = (255, 255, 0)
        # thickness = 1
        # cv2.line(color_map, obstacle_point, (rand_x, rand_y), color, thickness)

        # cv2.imshow('image', color_map)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

        does_obs_intersect = is_point_on_line_segment(
            obstacle_point,
            prev_random_point,
            (rand_x, rand_y)
        ) and does_point_interest_line(
            obstacle_point,
            prev_random_point,
            (rand_x, rand_y)
        )

        if does_obs_intersect:
            break

    # size = 4
    # color = (255, 255, 255)
    # thickness = -1
    # cv2.rectangle(color_map,
    #               (rand_x, rand_y),
    #               (rand_x + size, rand_x + size),
    #               color,
    #               thickness
    #               )
    #
    # for obstacle_point in obstacles:
    #     cv2.circle(color_map, obstacle_point, 1, (0, 0, 0), -1)

    if not does_obs_intersect:
        closest_point, dist = find_closest_point((rand_x, rand_y), list(all_random_points.keys()))
        all_random_points[closest_point].append((rand_x, rand_y, dist))

        # draw red square for the random point
        size = 4
        color = (0, 0, 255)
        thickness = -1
        cv2.rectangle(color_map,
                      (rand_x, rand_y),
                      (rand_x + size, rand_y + size),
                      color,
                      thickness
                      )

        color = (0, 255, 0)
        thickness = 1
        cv2.line(color_map, closest_point, (rand_x, rand_y), color, thickness)

        cv2.imshow('image', color_map)
        cv2.waitKey(4)
        cv2.destroyAllWindows()

        all_random_points[(rand_x, rand_y)] = [(rand_x, rand_y, 0)]
        prev_random_point = (rand_x, rand_y)

    prev_to_goal_intersection = False
    for obstacle_point in obstacles:
        prev_to_goal_intersection = is_point_on_line_segment(
            obstacle_point,
            prev_random_point,
            goal_pos
        ) and does_point_interest_line(
            obstacle_point,
            prev_random_point,
            goal_pos
        )
        if prev_to_goal_intersection:
            break

    if False and curr_iter == MAX_ITERS:
        print("Failed!")

        cv2.imshow('image', color_map)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        exit(0)

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

dijikstra_result = dijikstra(all_random_points, start_pos)

# Backtrack
back_current = goal_pos
while back_current != start_pos:
    _, parent_x, parent_y = dijikstra_result[back_current]

    color = (255, 0, 0)
    thickness = 2
    cv2.line(color_map, back_current, (parent_x, parent_y), color, thickness)

    back_current = (parent_x, parent_y)

# for obs in obstacles:
#     cv2.circle(color_map, obs, 1, (0, 0, 255), -1)

cv2.imshow('image', color_map)
cv2.waitKey(0)
cv2.destroyAllWindows()
