import numpy as np
from matplotlib import pyplot as plt


def line_x(start=-10, end=10, b=-1):
    x = np.linspace(start, end, 40)
    y = x * 0 + b
    return x, y


def line_y(start=-10, end=10, b=-1):
    y = np.linspace(start, end, 40)
    x = y * 0 + b
    return x, y

def generate_squre(a=5):
    list_x = []
    list_y = []
    x, y = line_x(-a, a, -a)
    for n, m in zip(x, y):
        list_x.append(n)
        list_y.append(m)

    x, y = line_y(-a, a, a)
    for n, m in zip(x, y):
        list_x.append(n)
        list_y.append(m)

    x, y = line_x(a, -a, a)
    for n, m in zip(x, y):
        list_x.append(n)
        list_y.append(m)

    x, y = line_y(a, -a, -a)
    for n, m in zip(x, y):
        list_x.append(n)
        list_y.append(m)

    list_square = []
    for i, j in zip(list_x, list_y):
        list_square.append([i, j])
    print(list_square)
    # return list_square

def circle(step=50, radius=30):
    alpha = np.linspace(0, 2 * np.pi, step)
    list_spiral = []
    r = radius
    x = np.round(r * np.cos(alpha))
    y = np.round(r * np.sin(alpha))
    for i, j in zip(x, y):
        list_spiral.append([i, j])

    plt.plot(x, y, "o")

    plt.show()
    print(list_spiral)


def main():
    generate_squre()
if __name__ == "__main__":
    main()
