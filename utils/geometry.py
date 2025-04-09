#! /usr/bin/env python3

import os
import json
import torch
import random
import pathlib
import numpy as np

import sys
sys.path.append(os.path.abspath(os.path.join(
    os.path.dirname(__file__), 
    '..',
    '..'
)))

from argparse import ArgumentParser
from PIL import Image
from typing import Literal, get_args


PointCoords = tuple[float, float]

def coords2XY(
    p: PointCoords,
    R: float
):
    """
    Extract a PointCoords instance's x and y coordinates.

    Params
    ------
        p: PointCoords, lat and lon of that point, resulted from CRS conversion.
        R: float, radius of Earth, in meters.

    Returns
    -------
        x, y: float, x and y coords in meters.
    """

    lat, lon = p[0], p[1]

    x = math.radians(lon) * R * math.cos(math.radians(lat))
    y = math.radians(lat) * R

    return x, y

def dist_2nodes(
    p1: PointCoords,
    p2: PointCoords
):
    """
    Calculate Haversine distance between 2 PointCoords points.

    Params
    ------
        p1, p2: PointCoords, lat and lon of that point, resulted from CRS conversion.

    Returns
    -------
        dist: float, distance in meters.
    """

    lat1, lon1 = p1[0], p1[1]
    lat2, lon2 = p2[0], p2[1]

    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)

    angle = math.sin(dlat / 2) ** 2 + \
            math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon / 2) ** 2

    dist = 2 * R * math.asin(math.sqrt(angle))

    return dist

def calAngleTriplePoints(
    p1: PointCoords,
    p2: PointCoords,
    p3: PointCoords
):

    a = coords2XY(p1)
    b = coords2XY(p2)
    c = coords2XY(p3)

    v1 = (
        a[0] - b[0],
        a[1] - b[1]
    )
    v2 = (
        c[0] - b[0],
        c[1] - b[1]
    )

    norm_v1 = math.hypot(*v1)
    norm_v2 = math.hypot(*v2)

    if (norm_v1 == 0) or (norm_v2 == 0):
        return 180

    dot_prod = v1[0] * v2[0] + v1[1] * v2[1]

    angle = math.degrees(
        math.acos(
            max(
                min(dot / (norm_v1 * norm_v2), 1.0), 
                -1.0
            )
        )
    )

    return angle