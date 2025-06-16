#! /usr/bin/env python3

import math
import itertools
from pprint import pprint
from lxml import etree
from .conversion import DEFAULT_GEOSTRING
from .geometry import PointCoords, dist_2nodes, calAngleTriplePoints
from pyproj import Transformer


def simplifyWayNodes(
    points: list[PointCoords],
    straight_angle_threshold: float = 175.0,
    min_segment_dist: float = 3.0,
    transformer = None
):
    """
    Simplify a list of points by removing points that are too close to each other
    or that form an angle that is too small.
    I took inspiration from Douglas-Peucker algorithm, FYI:
    https://en.wikipedia.org/wiki/Ramer-Douglas-Peucker_algorithm

    Params
    ------
        points: list[PointCoords], list of tuples (lat, lon) of the points to simplify.
        straight_angle_threshold: float, angle threshold, in degrees. Default 175.0.
        min_segment_dist: float, minimum distance between points, in meters. Default 3.0.

    Returns
    -------
        simplified_points: list[PointCoords], list of tuples (lat, lon) of the simplified points.
    """

    # No point in simplifying 2 nodes
    if (len(points) <= 2):
        return points

    # Keep first node
    simplified_points = [points[0]]
    # Last kept node
    last_kept = points[0]

    # Check angle, second first to second last nodes
    for i in range(1, len(points) - 1):

        this_angle = calAngleTriplePoints(
            points[i - 1],
            points[i],
            points[i + 1],
            transformer
        )
        this_dist = dist_2nodes(last_kept, points[i])

        if (
            (this_angle < straight_angle_threshold) and
            (this_dist >= min_segment_dist)
        ):
            simplified_points.append(points[i])
            last_kept = points[i]


    # Keep last node
    simplified_points.append(points[-1])

    # Fall back on sanity check
    if (len(simplified_points) < 2) and (len(points) >= 2):
        return [points[0], points[-1]]

    return simplified_points


def postprocessDownsamplingOSM(
    osm_root: etree.Element,
    straight_angle_threshold: float,
    min_segment_dist: float,
    geostring: str = DEFAULT_GEOSTRING
):
    """
    Postprocess OSM data by downsampling the nodes of the ways.

    Params
    ------
        osm_root: lxml.etree.Element, root element of the OSM XML file.
        straight_angle_threshold: float, angle threshold, in degrees.
        min_segment_dist: float, minimum distance between points, in meters.

    Returns
    -------
        osm_root: lxml.etree.Element, root element of the OSM XML file after downsampling.
    """

    transformer = Transformer.from_crs(
        DEFAULT_GEOSTRING,
        geostring,
        always_xy = True
    )

    # Map node ID to (lat, lon)
    nodes = {
        node.get("id"): (
            float(node.get("lat")), 
            float(node.get("lon")),
            float(next(
                (
                    tag.get("v") 
                    for tag in node.findall("tag") 
                    if tag.get("k") == "ele"
                ), 
                0
            ))
        ) 
        for node in osm_root.findall("node")
    }

    ways = osm_root.findall("way")
    new_node_id_gen = itertools.count(1_000_000)
    used_node_ids = set()
    new_nodes = []

    for way in ways:

        nd_refs = [
            nd.get("ref") 
            for nd in way.findall("nd")
        ]
        coords = [
            nodes[ref][:2]
            for ref in nd_refs 
            if ref in nodes
        ]

        if (len(coords) < 2):
            print(f"Skipping way {way.get('id')} cuz not enough points.")

        simplified = simplifyWayNodes(
            points = coords,
            straight_angle_threshold = straight_angle_threshold,
            min_segment_dist = min_segment_dist,
            transformer = transformer
        )
        
        if (len(simplified) < 2):
            print(f"Skipping way {way.get('id')} cuz its too simple after filtering.")
            continue
        
        # Remove old <nd>
        for nd in way.findall("nd"):
            way.remove(nd)

        # New <node> & <nd> refs

        for lat, lon in simplified:
            print(f"Using transformer: {transformer}")
            print(f"Before conversion: lon = {lon}, lat = {lat}")
            local_x, local_y = transformer.transform(lon, lat)
            print(f"After conversion: local_x = {local_x}, local_y = {local_y}")

            ele = next(
                (
                    nodes[ref][2] 
                    for ref in nd_refs 
                    if ((nodes[ref][0] == lat) and (nodes[ref][1] == lon))
                ),
                0.0
            )

            node_id = str(next(new_node_id_gen))

            if node_id not in used_node_ids:
                used_node_ids.add(node_id)

                node = etree.Element("node", id=node_id, visible="true", version="1", lat=str(lat), lon=str(lon))

                # tag_x = etree.Element("tag", k="local_x", v=f"{local_x:.4f}")
                # tag_y = etree.Element("tag", k="local_y", v=f"{local_y:.4f}")
                tag_ele = etree.Element("tag", k="ele", v=f"{ele:.4f}")

                # node.append(tag_x)
                # node.append(tag_y)
                node.append(tag_ele)

                new_nodes.append(node)

            nd = etree.Element("nd", ref = node_id)
            way.append(nd)

    # Remove old <node> elements
    for node in osm_root.findall("node"):
        osm_root.remove(node)

    # Add new resampled nodes
    for node in new_nodes:
        osm_root.append(node)
    print(f"[debug] final osm_root num nodes: {len(osm_root)}")

    return osm_root