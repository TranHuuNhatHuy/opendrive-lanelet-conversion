#! /usr/bin/env python3

import math
import itertools
from pprint import pprint
from lxml import etree
from geometry import PointCoords, dist_2nodes, coords2XY, calAngleTriplePoints


def simplifyWayNodes(
    points: list[PointCoords],
    straight_angle_threshold: float = 175.0,
    min_segment_dist: float = 3.0
):

    # No point in simplifying 2 nodes
    if (len(points) <= 2):
        return points

    # Keep first node
    simplified_points = [points[0]]

    # Check angle, second first to second last nodes
    for i in range(1, len(points) - 1):

        this_angle = calAngleTriplePoints(
            points[i - 1],
            points[i],
            points[i + 1]
        )
        this_dist = dist_2nodes(points[i - 1], points[i])

        if (
            (this_angle < straight_angle_threshold) or
            (this_dist >= min_segment_dist)
        ):
            simplified_points.append(points[i])

    # Keep last node
    simplified_points.append(points[-1])

    # Fall back on sanity check
    if (len(simplified_points) < 2) and (len(points) >= 2):
        return [points[0], points[-1]]

    return simplified_points


def postprocessDownsamplingOSM(
    osm_root,
    straight_angle_threshold: float,
    min_segment_dist: float
):

    # Map node ID to (lat, lon)
    nodes = {
        node.get("id"): (
            float(node.get("lat")), 
            float(node.get("lon"))
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
            nodes[ref] 
            for ref in nd_refs 
            if ref in nodes
        ]

        if (len(coords) < 2):
            print(f"Skipping way {way.get('id')} cuz not enough points.")

        simplified = simplifyWayNodes(coords, straight_angle_threshold)
        
        if (len(simplified) < 2):
            print(f"Skipping way {way.get('id')} cuz its too simple after filtering.")
            continue
        
        # Remove old <nd>
        for nd in way.findall("nd"):
            way.remove(nd)

        # New <node> & <nd> refs

        for lat, lon in simplified:

            node_id = str(next(new_node_id_gen))

            if node_id not in used_node_ids:
                used_node_ids.add(node_id)
                node = etree.Element(
                    "node", 
                    id = node_id, 
                    visible = "true", 
                    version = "1", 
                    lat = str(lat), 
                    lon = str(lon)
                )
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