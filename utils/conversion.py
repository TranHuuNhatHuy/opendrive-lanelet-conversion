#! /usr/bin/env python3

from crdesigner.common.config.lanelet2_config import lanelet2_config
from crdesigner.map_conversion.lanelet2.cr2lanelet import CR2LaneletConverter
from commonroad.scenario.scenario import Location, GeoTransformation
from crdesigner.map_conversion.map_conversion_interface import opendrive_to_commonroad


def prepConversionCRS(
    georeference_string: str = "EPSG:3857",
    x_translation: float | None = 0.0,
    y_translation: float | None = 0.0,
    z_rotation: float | None = None,
    scaling: float | None = None,
    geo_name_id: int | None = 11,
    gps_latitude: float | None = 0.0,
    gps_longitude: float | None = 0.0,
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
        georeference_string = georeference_string,
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