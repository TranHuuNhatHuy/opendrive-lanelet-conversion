import os
import sys
import csv
import shutil
from pathlib import Path
from lxml import etree
from pyproj import CRS, Transformer
import math
import itertools
from datetime import date

# Insert at the front of sys.path so the local repo is imported before any
# pip-installed crdesigner package.
# __file__ resolves correctly regardless of the working directory and regardless
# of whether opendrive-lanelet-conversion lives inside or alongside the repo.
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CR_DESIGNER_PATH = os.path.abspath(os.path.join(_SCRIPT_DIR, "..", "commonroad-scenario-designer"))
sys.path.insert(0, CR_DESIGNER_PATH)

from crdesigner.common.config.lanelet2_config import lanelet2_config
from crdesigner.common.config.opendrive_config import OpenDriveConfig
from crdesigner.map_conversion.lanelet2.cr2lanelet import CR2LaneletConverter
from commonroad.scenario.scenario import Location, GeoTransformation
from crdesigner.map_conversion.opendrive.odr2cr.opendrive_parser.parser import parse_opendrive
from crdesigner.map_conversion.opendrive.odr2cr.opendrive_conversion.network import Network
from crdesigner.map_conversion.opendrive.odr2cr.opendrive_conversion import network as odr_network_module

# Input handling
input_dir = Path("./sample_data")
set_list = [
    "naive",
    "CARLA",
    "esmini",
    "SafetyPool_Emil",
    "custom",
]

# Output handling
output_dir = Path(f"./output/{date.today().isoformat()}")
if not (os.path.exists(output_dir)):
    os.makedirs(output_dir)

# Downsampling params (will add to config later)
STRAIGHT_ANGLE_THRSH = 179.9            # Extremely strict angle threshold (trust me, 179 wasn't enough)
MIN_SEGMENT_DIST = 3.0                  # Minimum segment length accepted

PROJ_DEG = "EPSG:4326"                  # WGS84 (Degrees)
PROJ_MET = "EPSG:3857"                  # WGS64 (Meter)

PointCoords = tuple[float, float]
R = 6378000                             # Earth radius, in meters

# def extractGeorefString(xodr_path: str) -> tuple[str, bool]:
    
#     try:
#         with open(xodr_path, "rb") as f:
#             tree = etree.parse(f)
#             geo_elem = tree.find(".//geoReference")

#             if ((geo_elem is not None) and (geo_elem.text)):
#                 raw_proj_str = geo_elem.text.strip()
#                 print(f"Proj found in input: {raw_proj_str}")

#                 # Validate
#                 try:
#                     _ = CRS.from_proj4(raw_proj_str)
#                     return raw_proj_str, True
#                 except Exception as e:
#                     print(f"Invalid CRS string: {e}")

#     except Exception as e:
#         print(f"Error parsing geoReference from {xodr_path}: {e}")

#     print(f"Invalid proj, using default {PROJ_MET}")
#     return PROJ_MET, False

def prepConversionCRS(
    georeference_string: str = PROJ_MET,
    x_translation: float = 0.0,
    y_translation: float = 0.0,
    z_rotation: float = None,
    scaling: float = None,
    geo_name_id: int = 11,
    gps_latitude: float = 0.0,
    gps_longitude: float = 0.0,
    environment: str = None
):

    # Init scenario handling
    location_geotransformation = GeoTransformation(
        georeference_string,
        x_translation = x_translation,
        y_translation = y_translation,
        z_rotation = z_rotation,
        scaling = scaling
    )
    scenario_location = Location(
        geo_name_id = geo_name_id,
        gps_latitude = gps_latitude,
        gps_longitude = gps_longitude,
        geo_transformation = location_geotransformation,
        environment = environment
    )

    return scenario_location

def convertOpenDriveWithMapping(
    input_file: str,
    odr_conf: OpenDriveConfig,
):

    # Capture the OpenDRIVE-based lanelet id (stored in lanelet.description)
    # before the conversion utility strips it when creating a base LaneletNetwork.
    cr_lanelet_to_odr_lane = {}
    original_convert_to_base = odr_network_module.convert_to_base_lanelet_network

    def _capture_and_convert(conv_lanelet_network):
        for lanelet in conv_lanelet_network.lanelets:
            odr_encoded = getattr(lanelet, "description", None)
            if odr_encoded is not None:
                cr_lanelet_to_odr_lane[int(lanelet.lanelet_id)] = str(odr_encoded)
        return original_convert_to_base(conv_lanelet_network)

    odr_network_module.convert_to_base_lanelet_network = _capture_and_convert
    try:
        opendrive = parse_opendrive(Path(input_file))
        road_network = Network()
        road_network.load_opendrive(opendrive)
        scenario = road_network.export_commonroad_scenario(od_config=odr_conf)
    finally:
        odr_network_module.convert_to_base_lanelet_network = original_convert_to_base

    return scenario, cr_lanelet_to_odr_lane


def buildCrToLl2LaneMapping(converter: CR2LaneletConverter):

    if (converter is None) or (converter.osm is None):
        return {}

    lanelet_way_rel_ids = {
        (way_rel.left_way, way_rel.right_way): way_rel_id
        for way_rel_id, way_rel in converter.osm.way_relations.items()
        if way_rel.tag_dict.get("type") == "lanelet"
    }

    cr_to_ll2 = {}
    for cr_lanelet_id, left_way_id in converter.left_ways.items():
        right_way_id = converter.right_ways.get(cr_lanelet_id)
        if right_way_id is None:
            continue

        ll2_relation_id = lanelet_way_rel_ids.get((left_way_id, right_way_id))
        if ll2_relation_id is not None:
            cr_to_ll2[int(cr_lanelet_id)] = int(ll2_relation_id)

    return cr_to_ll2


def parseOdrLaneId(odr_encoded_lane_id: str):

    # Expected formats include: road.section.lane.width and road.section.lane.width.m
    parts = str(odr_encoded_lane_id).split(".")
    if len(parts) < 3:
        return None

    return parts[0], parts[1], parts[2]

def coords2XY(
    p: PointCoords,
    # transformer
):

    lat, lon = p[0], p[1]

    x = math.radians(lon) * R * math.cos(math.radians(lat))
    y = math.radians(lat) * R

    # x, y = transformer.transform(lon, lat)

    return x, y

def dist_2nodes(
    p1: PointCoords,
    p2: PointCoords
):

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
    p3: PointCoords,
    # transformer
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
                min(
                    dot_prod / (norm_v1 * norm_v2), 
                    1.0
                ), 
                -1.0
            )
        )
    )

    return angle

def simplifyWayNodes(
    points: list[PointCoords],
    straight_angle_threshold: float = 175.0,
    min_segment_dist: float = 3.0,
    # transformer = None
):

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
            # transformer
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
):

    transformer = Transformer.from_crs(
        PROJ_DEG,
        PROJ_MET,
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
            # transformer = transformer
        )
        
        if (len(simplified) < 2):
            print(f"Skipping way {way.get('id')} cuz its too simple after filtering.")
            continue
        
        # Remove old <nd>
        for nd in way.findall("nd"):
            way.remove(nd)

        # New <node> & <nd> refs

        for lat, lon in simplified:
            local_x, local_y = transformer.transform(lon, lat)
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

                node = etree.Element("node", id=node_id, visible="true", version="1", lat="", lon="")

                tag_x = etree.Element("tag", k="local_x", v=f"{local_x:.4f}")
                tag_y = etree.Element("tag", k="local_y", v=f"{local_y:.4f}")
                tag_ele = etree.Element("tag", k="ele", v=f"{ele:.4f}")

                node.append(tag_x)
                node.append(tag_y)
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

    # Finally, switch generator to VMB
    osm_root.set("generator", "VMB")

    return osm_root

# Config with lane-section merging disabled — used for the "custom" set so that
# individual lane sections are preserved as separate lanelets in the output.
no_concat_config = OpenDriveConfig()
no_concat_config.concatenate_lanelets_flag = False

process_time_log = {}

for set_name in set_list:

    process_time_log[set_name] = {}

    print(f"\n=============== Working on {set_name} ===============\n")

    this_set_input_path = input_dir / set_name
    this_set_output_path = output_dir / set_name
    if not (os.path.exists(this_set_output_path)):
        os.makedirs(this_set_output_path)

    # Use no-concat config for the custom set to preserve lane section boundaries
    odr_conf = no_concat_config

    for input_file in os.listdir(this_set_input_path):

        start_moment = os.times()

        # Input handling
        input_file_path = this_set_input_path / input_file
        print(f"\nConverting {input_file_path}")

        # Output handling
        input_file_tail_trimmed = ".".join(input_file.split(".")[ : -1])
        output_filename = f"converted_{input_file_tail_trimmed}.osm"
        output_path = this_set_output_path / output_filename

        # proj_str, output_latlon = extractGeorefString(input_file_path)
        scenario_location = prepConversionCRS()
        
        # Conversion
        converted_osm = None
        converter = None
        cr_lanelet_to_odr_lane = {}
        try:
            scenario, cr_lanelet_to_odr_lane = convertOpenDriveWithMapping(
                input_file = input_file_path,
                odr_conf = odr_conf,
            )
            scenario.location = scenario_location
            l2osm = CR2LaneletConverter(lanelet2_config)
            converted_osm = l2osm(scenario)
            converter = l2osm
        except Exception as e:
            print(f"Error during conversion: {e}")

        # Conversion succeed!
        if (converted_osm is not None):

            done_conv_moment = os.times()
            done_conv_time_secs = done_conv_moment.elapsed - start_moment.elapsed

            predown_filename = f"predown_{input_file_tail_trimmed}.osm"
            predown_path = this_set_output_path / predown_filename
            with open(predown_path, "wb") as file_out:
                file_out.write(etree.tostring(
                    converted_osm, 
                    xml_declaration = True, 
                    encoding = "UTF-8", 
                    pretty_print = True
                ))

            # Save OpenDrive -> Lanelet2 ID mapping as CSV

            start_mapping_moment = os.times()

            mapping_filename = f"id_mapping_{input_file_tail_trimmed}.csv"
            mapping_path = this_set_output_path / mapping_filename
            with open(mapping_path, "w", newline="") as csv_file:
                writer = csv.writer(csv_file)
                writer.writerow([
                    "opendrive_road_id",
                    "opendrive_section_id",
                    "opendrive_lane_id",
                    "lanelet2_relation_id",
                ])

                cr_to_ll2 = buildCrToLl2LaneMapping(converter)
                mapping_rows = []
                for cr_lanelet_id, ll2_relation_id in cr_to_ll2.items():
                    odr_encoded_lane_id = cr_lanelet_to_odr_lane.get(cr_lanelet_id)
                    if odr_encoded_lane_id is None:
                        continue

                    odr_triplet = parseOdrLaneId(odr_encoded_lane_id)
                    if odr_triplet is None:
                        continue

                    road_id, section_id, lane_id = odr_triplet
                    mapping_rows.append((road_id, section_id, lane_id, ll2_relation_id))

                def _sort_key(row):
                    road, section, lane, relation = row
                    try:
                        return (int(road), int(section), int(lane), int(relation))
                    except ValueError:
                        return (road, section, lane, relation)

                for road_id, section_id, lane_id, ll2_relation_id in sorted(mapping_rows, key=_sort_key):
                    writer.writerow([road_id, section_id, lane_id, ll2_relation_id])

            print(f"ID mapping saved to {mapping_path} ({len(mapping_rows)} entries)")

            done_mapping_moment = os.times()
            done_mapping_time_secs = done_mapping_moment.elapsed - start_mapping_moment.elapsed

            # Here comes my postprocessing
            downsamp_osm = postprocessDownsamplingOSM(
                converted_osm, 
                STRAIGHT_ANGLE_THRSH,
                MIN_SEGMENT_DIST,
            )

            done_downsamp_moment = os.times()
            done_downsamp_time_secs = done_downsamp_moment.elapsed - done_mapping_moment.elapsed

            with open(output_path, "wb") as file_out:
                file_out.write(etree.tostring(
                    downsamp_osm, 
                    xml_declaration = True, 
                    encoding = "UTF-8", 
                    pretty_print = True
                ))
        
            print(f"Converted file saved to : {output_path}")
            print(f"Conversion time: {done_conv_time_secs:.2f} secs")
            print(f"Mapping time: {done_mapping_time_secs:.2f} secs")
            print(f"Downsampling time: {done_downsamp_time_secs:.2f} secs")
            print(f"Total processing time: {(done_downsamp_moment.elapsed - start_moment.elapsed):.2f} secs")

            process_time_log[set_name][input_file] = {
                "conversion_time_secs": done_conv_time_secs,
                "mapping_time_secs": done_mapping_time_secs,
                "downsampling_time_secs": done_downsamp_time_secs,
                "total_time_secs": done_downsamp_moment.elapsed - start_moment.elapsed
            }

        else:
            print(f"Conversion failed for {input_file_path}, skipping postprocessing and output.")
            process_time_log[set_name][input_file] = None


# Save processing times log as CSV
times_log_path = output_dir / "processing_times_log.csv"
with open(
    times_log_path, "w", 
    newline = ""
) as csv_file:
    writer = csv.writer(csv_file)
    writer.writerow([
        "set_name",
        "input_file",
        "conversion_time_secs",
        "mapping_time_secs",
        "downsampling_time_secs",
        "total_time_secs"
    ])

    for set_name, file_times in process_time_log.items():
        for input_file, times in file_times.items():
            if (times is not None):
                writer.writerow([
                    set_name,
                    input_file,
                    f"{times['conversion_time_secs']:.2f}",
                    f"{times['mapping_time_secs']:.2f}",
                    f"{times['downsampling_time_secs']:.2f}",
                    f"{times['total_time_secs']:.2f}"
                ])
            else:
                writer.writerow([
                    set_name, input_file, 
                    "conversion_failed", "", "", ""
                ])