import math
import cv2
from cv2 import norm
import numpy as np
from graph import Graph

# user defined parameters
meter_per_px = 0.0253929866989117 # m / px
px_per_meter = 1.0 / meter_per_px # px / m
discretization_distance = 1 # m
doors_size = 1.3 # m
robot_radius = 0.2 # m
image_path = 'input.png'


def meter_2_pixel(p):
    return int(round(p[0] * px_per_meter)), int(round(p[1] * px_per_meter))


def pixel_2_meter(p):
    return p[0] * meter_per_px, p[1] * meter_per_px


def index_2_meter(p):
    return p[0] * discretization_distance, p[1] * discretization_distance


def meter_2_index(p):
    return int(round(p[0] / discretization_distance)), int(round(p[1] / discretization_distance))


def index_2_pixel(p):
    p = index_2_meter(p)
    return meter_2_pixel(p)


def pixel_2_index(p):
    p = pixel_2_meter(p)
    return meter_2_index(p)


def compute_center(points):
    sum_x = 0
    sum_y = 0

    for point in points:
        sum_x += point[0]
        sum_y += point[1]

    cx = sum_x / len(points)
    cy = sum_y / len(points)

    return cx, cy


def distance(p1, p2):
    return math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)


def reverse(p):
    return p[1], p[0]


def main():
    # reading image
    img = cv2.imread(image_path)
    img = cv2.bitwise_not(img)

    # converting image into grayscale image
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # setting threshold of gray image
    _, threshold = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

    # close operation to highlight the contours of the rooms
    k = int(np.round(doors_size * px_per_meter))
    closed = cv2.morphologyEx(threshold, cv2.MORPH_CLOSE, np.ones((k, k), np.uint8))

    # dilate operation to inflate the walls to consider the robot as a point
    k = int(np.round(robot_radius * px_per_meter))
    walls = cv2.morphologyEx(threshold, cv2.MORPH_DILATE, np.ones((k, k), np.uint8))

    # room contours
    contours, _ = cv2.findContours(closed, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    rooms = []

    i = 0
    for contour in contours:
        # here we are ignoring first counter because
        # findcontour function detects whole image as shape
        if i == 0:
            i = 1
            continue

        # cv2.approxPloyDP() function to approximate the shape
        approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True)

        # room
        if len(approx) == 4:
            cv2.drawContours(img, [contour], 0, (0, 255, 0), 2)
            rooms.append(approx)
        # corridor
        else:
            cv2.drawContours(img, [contour], 0, (0, 0, 255), 2)

    # list of doors contours
    doors_img = cv2.subtract(closed, walls)

    doors_contours, _ = cv2.findContours(doors_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    doors = []
    
    for door_contour in doors_contours:
        door_approx = cv2.approxPolyDP(door_contour, 0.01 * cv2.arcLength(door_contour, True), True)
        doors.append(door_approx)

    # matrix with graph nodes
    # 1 node present
    # 0 node not present
    nodes = np.ones(pixel_2_index(gray.shape), np.uint8)

    # cancel nodes that are inside walls or rooms
    for i in range(0, nodes.shape[0]):
        for j in range(0, nodes.shape[1]):
            pixel = index_2_pixel((i, j))

            # check if inside a wall
            if walls[pixel] == 255:
                nodes[i, j] = 0

            # check if inside a rectangle
            for room in rooms:
                if (cv2.pointPolygonTest(room, reverse(pixel), False) >= 0):
                    nodes[i, j] = 0

    # create graph
    graph = Graph()
    
    # add nodes to the graph
    # connect two nodes if a line connecting them does not intersect any wall

    for i in range(0, nodes.shape[0]):
        for j in range(0, nodes.shape[1]):
            if (nodes[i, j] != 1):
                continue

            pixel = index_2_pixel((i, j))

            neighbours = []

            # 4-connected point's neighbours
            if i > 0:
                neighbours.append((i - 1, j))
            if i < nodes.shape[0] - 1:
                neighbours.append((i + 1, j))
            if j > 0:
                neighbours.append((i, j - 1))
            if j < nodes.shape[1] - 1:
                neighbours.append((i, j + 1))

            # crop the image around a node big with size 2 step of the graph
            # if near the border of the image, leave out the part towards the border
            min_x = pixel[0]
            if i > 0:
                min_x = pixel[0] - round(discretization_distance * px_per_meter) + 1

            max_x = pixel[0]
            if i < nodes.shape[0] - 1:
                max_x = pixel[0] + round(discretization_distance * px_per_meter)

            min_y = pixel[1]
            if j > 0:
                min_y = pixel[1] - round(discretization_distance * px_per_meter) + 1

            max_y = pixel[1]
            if j < nodes.shape[1] - 1:
                max_y = pixel[1] + round(discretization_distance * px_per_meter)

            cropped_walls = walls[min_x : max_x, min_y : max_y]

            for neighbour in neighbours:
                # check if neighbour is a node
                if nodes[neighbour[0], neighbour[1]] == 1:
                    neighbour_pixel = index_2_pixel(neighbour)

                    # clean the edges image
                    edges = np.copy(cropped_walls) * 0

                    # p1 is the node typically in the center of the image
                    p1 = (- min_x + pixel[0], - min_y + pixel[1])

                    # p2 is the neighbour node
                    p2 = (neighbour_pixel[0] - pixel[0] + p1[0],
                            neighbour_pixel[1] - pixel[1] + p1[1])

                    # draw a line between the two nodes
                    cv2.line(edges, reverse(p1), reverse(p2), 255, 1)

                    # if there are no walls between the two nodes add an edge
                    if cv2.countNonZero(cv2.bitwise_and(edges, cropped_walls)) == 0:
                        a = index_2_meter((i, j))
                        b = index_2_meter(neighbour)

                        if (a not in graph.vertices):
                            graph.add_vertex(a)

                        if (b not in graph.vertices):
                            graph.add_vertex(b)

                        graph.add_edge(a, b)

    # for each door find the nearest room, find the nearest graph vertex wrt door center, compute the normal to the door's long side
    # add a vertex in the direction of the center of the room starting from the door at a distance of discretization_distance
    for door in doors:
        door = door[:, 0]
        door = [pixel_2_meter(reverse(p)) for p in door]

        door_center = compute_center(door)

        # find nearest room with respect to door center
        nearest_room_center = (-1, -1)
        min_room_dist = math.inf

        for room in rooms:
            room = room[:, 0]
            room = [pixel_2_meter(reverse(p)) for p in room]

            room_center = compute_center(room)

            min_x, min_y = map(min, zip(*room))
            max_x, max_y = map(max, zip(*room))

            dx = max(min_x - door_center[0], 0, door_center[0] - max_x)
            dy = max(min_y - door_center[1], 0, door_center[1] - max_y)
            dist = math.sqrt(dx ** 2 + dy ** 2)

            if dist < min_room_dist:
                min_room_dist = dist
                nearest_room_center = room_center

        # find the nearest graph node with respect to the door center
        min_vertex_dist = math.inf
        near_vertex = (-1, -1)
        for key, _ in graph.vertices.items():
            dist = distance(door_center, key)
            if dist < min_vertex_dist:
                min_vertex_dist = dist
                near_vertex = key

        # search the long side of the door rectangle
        side_length = 0
        line = [(-1, -1), (-1, -1)]
        for i in range(0, len(door)):
            lenght = distance(door[i], door[(i+1) % 4])
            if lenght > side_length:
                side_length = lenght
                line = [door[i], door[(i+1) % 4]]

        # find normal to the long side of the door
        x1, y1 = line[0]
        x2, y2 = line[1]

        dx = x2 - x1
        dy = y2 - y1

        n1 = (-dy, dx)
        n2 = (dy, -dx)

        normalizer = 1.0 / np.linalg.norm(n1)

        n1 = (n1[0] * normalizer, n1[1] * normalizer)
        n2 = (n2[0] * normalizer, n2[1] * normalizer)

        # check if room center is above or below the line
        v1 = (x2 - x1, y2 - y1)   # Vector 1
        v2 = (x2 - nearest_room_center[0], y2 - nearest_room_center[1])   # Vector 2
        xp = v1[0] * v2[1] - v1[1] * v2[0]  # Cross product

        normal = n1

        if xp > 0:
            normal = n2
        elif xp < 0:
            normal = n1
        else:
            continue

        # create new vertex inside the room
        new_point = (door_center[0] + normal[0] * discretization_distance,
                     door_center[1] + normal[1] * discretization_distance)

        # add new vertex to the graph
        if near_vertex != (-1, -1):
            graph.add_vertex(new_point)
            graph.add_edge(new_point, near_vertex)
        else:
            print("ERROR")

    # print graph
    for key, values in graph.vertices.items():
        p1 = meter_2_pixel(key)
        # vertex
        cv2.circle(img, reverse(p1), 1, (0, 0, 255), 3)

        for value in values:
            p2 = meter_2_pixel(value)
            # edge
            cv2.line(img, reverse(p1), reverse(p2), (255, 0, 255), 1)

    # write image
    cv2.imwrite('output.png', img)


if __name__ == "__main__":
    main()
