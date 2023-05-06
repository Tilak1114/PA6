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

gray_map = cv2.imread("maps/" + "map2" + ".png", cv2.IMREAD_GRAYSCALE)

color_map = cv2.imread("maps/" + "map2" + ".png")

gray_map = np.array(gray_map)

all_random_points = {start_pos: [start_pos]}

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


def does_point_interest_line(point, line_start, line_end):
    x1, y1 = line_start
    x2, y2 = line_end
    if x2 - x1 == 0:
        return point[0] == x1 and y1 <= point[1] <= y2 or y2 <= point[1] <= y1
    m = (y2 - y1) / (x2 - x1)
    b = y1 - m * x1

    expected_y = m * point[0] + b

    return abs(point[1] - expected_y) < 2


def find_closest_point(point, points):
    closest_dist = math.inf
    closest_point = points[0]
    for p in points:
        current_dist = np.linalg.norm(np.array(point) - np.array(p))
        if current_dist < closest_dist:
            closest_dist = current_dist
            closest_point = p

    return closest_point


point_radius = len(gray_map)
# point_radius = 300

curr_iter = 0

# curr_iter < MAX_ITERS
while True:
    curr_iter += 1
    radius_x_start = max(0, prev_random_point[0] - point_radius)
    radius_x_end = min(len(gray_map), prev_random_point[0] + point_radius)
    radius_y_start = max(0, prev_random_point[1] - point_radius)
    radius_y_end = min(len(gray_map[0]), prev_random_point[1] + point_radius)

    rand_x = np.random.randint(radius_x_start, radius_x_end)
    rand_y = np.random.randint(radius_y_start, radius_y_end)

    does_intersect = False
    for obstacle_point in obstacles:

        size = 4
        color = (0, 0, 255)
        thickness = -1
        cv2.rectangle(color_map,
                      (rand_x, rand_y),
                      (rand_x + size, rand_y + size),
                      color,
                      thickness
                      )

        cv2.circle(color_map, obstacle_point, 1, (0, 0, 255), -1)

        cv2.imshow('image', color_map)
        cv2.waitKey(1)
        cv2.destroyAllWindows()

        does_intersect = does_point_interest_line(
            obstacle_point,
            prev_random_point,
            (rand_x, rand_y)
        )
        if does_intersect:
            break

    size = 4
    color = (255, 255, 255)
    thickness = -1
    cv2.rectangle(color_map,
                  (rand_x, rand_y),
                  (rand_x + size, rand_y + size),
                  color,
                  thickness
                  )

    if does_intersect:
        for obstacle_point in obstacles:
            cv2.circle(color_map, obstacle_point, 1, (0, 0, 0), -1)

    if not does_intersect:
        closest_point = find_closest_point((rand_x, rand_y), list(all_random_points.keys()))
        all_random_points[closest_point].append((rand_x, rand_y))

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

        all_random_points[(rand_x, rand_y)] = []
        prev_random_point = (rand_x, rand_y)

    prev_to_goal_intersection = False
    for obstacle_point in obstacles:
        prev_to_goal_intersection = does_point_interest_line(
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

# for obs in obstacles:
#     cv2.circle(color_map, obs, 1, (0, 0, 255), -1)

cv2.imshow('image', color_map)
cv2.waitKey(0)
cv2.destroyAllWindows()
