#! /usr/bin/env python3

from crdesigner.common.config.lanelet2_config import lanelet2_config
from crdesigner.map_conversion.lanelet2.cr2lanelet import CR2LaneletConverter
from commonroad.scenario.scenario import Location, GeoTransformation
from crdesigner.map_conversion.map_conversion_interface import opendrive_to_commonroad


def prepConversionCRS(
    georeference_string: str = "EPSG:3857",
    x_translation: float = 0.0,
    y_translation: float = 0.0,
    z_rotation: float = None,
    scaling: float = None,
    geo_name_id: int = 11,
    gps_latitude: float = 0.0,
    gps_longitude: float = 0.0,
    environment: str = None
):
    """
    Prep the CRS conversion instance scenario_location.

    Params
    ------
        georeference_string: str, georeference string to be used for conversion.
        x_translation: float, x translation in meters.
        y_translation: float, y translation in meters.
        z_rotation: float, rotation in degrees.
        scaling: float, scaling factor.
        geo_name_id: int, geo name id for the scenario location.
        gps_latitude: float, GPS latitude of the scenario location.
        gps_longitude: float, GPS longitude of the scenario location.
        environment: str, environment for the scenario location.

    Returns
    -------
        scenario_location: Location, the scenario location object with the specified parameters.
    """

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

def conductConversion(
    input_file: str,
    scenario_location: Location,
):
    
    try:

        # Scenario initialization
        scenario = opendrive_to_commonroad(input_file)
        scenario.location = scenario_location

        # Attempt conversion
        if (scenario):
            l2osm = CR2LaneletConverter(lanelet2_config)
            osm = l2osm(scenario)
            return osm

    except Exception as e:
        print(f"Error during conversion: {e}")
        
    return None