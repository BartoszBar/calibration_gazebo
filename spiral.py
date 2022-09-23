import numpy as np
from matplotlib import pyplot as plt


def line_x(start=-10, end=10, b=-1, iter1=40):
    x = np.linspace(start, end, iter1)
    y = x * 0 + b
    distance = end - start
    return x, y, iter1, np.abs(distance)


def line_y(start=-10, end=10, b=-1, iter1=40):
    y = np.linspace(start, end, iter1)
    x = y * 0 + b
    distance = end - start
    return x, y, iter1, np.abs(distance)


def generate_squre(b=5):
    a = b / 2
    list_iter_distance = []
    list_square = []
    move_plan = [[-a, a, -a], [-a, a, a], [a, -a, a], [a, -a, -a]]
    for i in range(len(move_plan)):
        current_plan = move_plan[i]
        if i % 2 == 0:
            x, y, iter1, distance = line_x(current_plan[0], current_plan[1], current_plan[2])
        else:
            x, y, iter1, distance = line_y(current_plan[0], current_plan[1], current_plan[2])
        list_iter_distance.append([iter1, distance])
        for n, m in zip(x, y):
            list_square.append([n, m])
    iter_sum, dist_sum = 0, 0
    for i in list_iter_distance:
        iter_sum += i[0]
        dist_sum += i[1]

    return list_square, iter_sum, dist_sum


def count_rospy_rate(v=2, iterations_move=100, s=5):
    distance_per_iter = s / iterations_move
    f = v / distance_per_iter
    return f


def circle(iter1=50, radius=30):
    alpha = np.linspace(0, 2 * np.pi, iter1)
    list_spiral = []
    r = radius
    x = np.round(r * np.cos(alpha))
    y = np.round(r * np.sin(alpha))
    for i, j in zip(x, y):
        list_spiral.append([i, j])
    return list_spiral, iter1, 2 * np.pi * radius


def main():
    a, b, distance = generate_squre()
    print(b, distance)
    # print(b, distance)


if __name__ == "__main__":
    main()
