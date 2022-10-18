import numpy as np


def _compute_bspline_bases(x, n):
    x = max(x, 0)
    x = min(x, n - 2)
    # position in basis function

    v = np.zeros((n, 1))
    b = np.zeros((n, 1))

    for i in range(n):
        v[i, 0] = x - i + 2
        vv = v[i, 0]

        if vv < 0:
            b[i, 0] = 0
        elif vv < 1:
            b[i, 0] = 0.5 * vv ** 2
        elif vv < 2:
            b[i, 0] = 0.5 * (-3 + 6 * vv - 2 * vv ** 2)
        elif vv < 3:
            b[i, 0] = 0.5 * (3 - vv) ** 2
        else:
            b[i, 0] = 0
    return b


def dynamic_Bspline(x, points):
    """

    :param x:
    :param points:
    :return:
    """
    n, _ = points.shape
    b = _compute_bspline_bases(x, n)
    xx = b.T @ points[:, 0]
    yy = b.T @ points[:, 1]
    return xx, yy


def dynamic_Bspline_radius(x, radii):
    """

    :param x:
    :param radii:
    :return:
    """
    n, _ = radii.shape
    b = _compute_bspline_bases(x, n)
    rr = b.T @ radii
    return rr


# def dynamic_BsplineRadius_test(x, radii):
#     """
#
#     :param x:
#     :param radii:
#     :return:
#     """
#     n = radii.shape
#     b = _compute_bspline_bases(x, n[0])
#     rr = b.T @ radii
#     return rr


def dynamic_Bspline_tangent(x, points):
    """

    :param x:
    :param points:
    :return:
    """
    n, _ = points.shape
    x = max(x, 0)
    x = min(x, n - 2)
    # position in basis function

    v = np.zeros((n, 1))
    b = np.zeros((n, 1))

    for i in range(n):
        v[i, 0] = x - i + 2
        vv = v[i, 0]
        if vv < 0:
            b[i, 0] = 0
        elif vv < 1:
            b[i, 0] = vv
        elif vv < 2:
            b[i, 0] = 3 - 2 * vv
        elif vv < 3:
            b[i, 0] = -3 + vv
        else:
            b[i, 0] = 0

    xx = b.T @ points[:, 0]
    yy = b.T @ points[:, 1]
    norm = (xx ** 2 + yy ** 2) ** 0.5
    xx = xx / norm
    yy = yy / norm
    return xx, yy


def dynamic_Bspline_normal(x, points):
    """

    :param x:
    :param points:
    :return:
    """
    yy, xx = dynamic_Bspline_tangent(x, points)
    yy = -yy
    return xx, yy
