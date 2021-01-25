#!/usr/bin/env python
"""
Utilities for various geodetic conversions. Adapted from https://kb.unavco.org/kb/article/trimble-netr9-receiver-gsof-messages-806.html
"""


from math import sin, cos, sqrt, atan2, degrees, radians


def llh2xyz(lat, lon, h, a=6378137.0, finv=298.257223563, input_degrees=True):
    """
    Converts ellipsoidal coordinates to cartesian.

    :param lat:
    :param lon:
    :param h:

    :return: A tuple containing cartesian coordinates in ECEF [X, Y, Z] meters.
    """
    # convert degrees to radians if necessary
    if input_degrees:
        lat = radians(lat)
        lon = radians(lon)
    # Default (a=6378137.0, finv=298.257223563) is for WGS84
    f = 1/finv
    b = a*(1 - f)
    e2 = 1 - (1 - f)**2
    # Compute the Cartesian coordinates
    v = a/sqrt(1-e2*sin(lat)*sin(lat))
    x = (v+h)*cos(lat)*cos(lon)
    y = (v+h)*cos(lat)*sin(lon)
    z = (v*(1-e2)+h)*sin(lat)
    return x, y, z

def xyz2llh(x, y, z, a=6378137.0, finv=298.257223563, output_degrees=True):
    """
    Converts cartesian coordinates to ellipsoidal. Uses an iterative algorithm.

    :param x: x coordinate in meters
    :param y: y coordinate in meters
    :param z: z coordinate in meters
    :param a: major semi-axis (Default: WGS84)
    :param finv: inverse of flattening (Default: WGS84)
    :param output_degrees: A boolean to select output in degrees (default is True)
    :return: A tuple containing geodetic coordinates in WGS84 (lat, lon, h).

    WGS84: a=6378137.0, finv=298.257223563
    """
    # Default (a=6378137.0, finv=298.257223563) is for WGS84
    f = 1/finv
    b = a*(1 - f)
    e2 = 1 - (1 - f)**2
    # Latitude and height convergence criteria
    elat = 1.0e-12
    eht = 1.e-5
    # Initial values for iteration
    p = sqrt(x*x + y*y)
    lat = atan2(z, p*(1-e2))
    h = 0
    dh = 1
    dlat = 1
    # Iterate until lat & h converge to elat & eht
    while dlat > elat or dh > eht:
        lat0 = lat
        h0 = h
        v = a/sqrt(1-e2*sin(lat)*sin(lat))
        h = p/cos(lat)-v
        lat = atan2(z, p*(1-e2*v/(v+h)))
        dlat = abs(lat-lat0)
        dh = abs(h-h0)
    lon = atan2(y, x)
    if output_degrees:
        # output degrees
        geodetic_coordinates = (degrees(lat), degrees(lon), h)
    else:
        # output radians
        geodetic_coordinates = (lat, lon, h)
    return geodetic_coordinates


def xyz2neu(X, Y, Z, x, y, z):
    """
    Rotates the <(x-X),(y-Y),(z-Z)> vector into a topocentric coordinate system
    tangential to the reference point XYZ.

    Assumes coordinates are given in meters.
    :param X: Reference Coordinate X (meters)
    :param Y: Reference Coordinate Y (meters)
    :param Z: Reference Coordinate Z (meters)
    :param x: Observed location x (meters)
    :param y: Observed location y (meters)
    :param z: Observed location z (meters)
    :return: A tuple containing topocentric coordinates in n, e, u (meters)

    WGS84: a=6378137.0, finv=298.257223563
    """
    lat, lon, h = xyz2llh(X, Y, Z, a=6378137.0, finv=298.257223563, output_degrees=False)
    e = -sin(lon) * (x - X) + cos(lon) * (y - Y)
    n = -sin(lat) * cos(lon) * (x - X) - sin(lat) * sin(lon) * (y - Y) + cos(lat) * (z - Z)
    u = cos(lat) * cos(lon) * (x - X) + cos(lat) * sin(lon) * (y - Y) + sin(lat) * (z - Z)
    return n, e, u


def xyz_to_globk_neu(X, Y, Z, x, y, z):
    """
    Finds the arc-length between two LLH points in dN and dE.
    The parameter dU is defined as the difference between the observed and reference ellipsoidal height.

    Assumes coordinates are given in meters.
    :param X: Reference Coordinate X (meters)
    :param Y: Reference Coordinate Y (meters)
    :param Z: Reference Coordinate Z (meters)
    :param x: Observed location x (meters)
    :param y: Observed location y (meters)
    :param z: Observed location z (meters)
    :return: A tuple containing n, e, u (meters)
    """

    def sub(a, b):
        """
        Subtracts elements in two lists
        """
        c = [i - j for i, j in zip(a, b)]
        return c

    def mul(a, b):
        """
        Multiplies elements in two lists
        """
        c = [i * j for i, j in zip(a, b)]
        return c

    P1 = xyz2llh(X, Y, Z, a=6378137.0, finv=298.257223563, output_degrees=False)
    P2 = xyz2llh(x, y, z, a=6378137.0, finv=298.257223563, output_degrees=False)
    # Subtract the reference coordinate from the observed coordinate
    P_diff = sub(P2, P1)
    # Convert the dLLH to dNEU
    # Assumes a sphere
    rot = [6378137.0, 6378137.0*cos(P1[0]), 1]
    NEU = mul(P_diff, rot)

    return tuple(NEU)


def covrot(X, Y, Z, cxx, cyy, czz, cxy, cxz, cyz):
    """
    Rotates the sigmas and the correlations from xyz to enu
    :param cxx:
    :param cyy:
    :param czz:
    :param cxy:
    :param cxz:
    :param cyz:
    :return:
    """
    def matmult(a, b):
        zip_b = zip(*b)
        return [[sum(ele_a*ele_b for ele_a, ele_b in zip(row_a, col_b)) for col_b in zip_b] for row_a in a]

    lat, lon, h = xyz2llh(X, Y, Z, a=6378137.0, finv=298.257223563, output_degrees=False)

    rxx = -sin(lat)*cos(lon)
    rxy = -sin(lat)*sin(lon)
    rxz = cos(lat)
    ryx = -sin(lon)
    ryy = cos(lon)
    ryz = 0
    rzx = cos(lat)*cos(lon)
    rzy = cos(lat)*sin(lon)
    rzz = sin(lat)

    rotation_matrix = [[rxx, rxy, rxz],
                       [ryx, ryy, ryz],
                       [rzx, rzy, rzz]]
    rotation_transpose = [[rxx, ryx, rzx],
                          [rxy, ryy, rzy],
                          [rxz, ryz, rzz]]
    covariance_matrix = [[cxx**2, cxy*cxx*cyy, cxz*cxx*czz],
                         [cxy*cxx*cyy, cyy**2, cyz*cyy*czz],
                         [cxz*cxx*czz, cyz*cyy*czz, czz**2]]

    tmp = matmult(rotation_matrix, covariance_matrix)

    result = matmult(tmp, rotation_transpose)

    cxx = result[0][0]**0.5
    cyy = result[1][1]**0.5
    czz = result[2][2]**0.5
    cxy = result[0][1]/(cxx*cyy)
    cxz = result[0][2]/(cxx*czz)
    cyz = result[1][2]/(cyy*czz)

    return cxx, cyy, czz, cxy, cxz, cyz