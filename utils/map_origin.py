#! /usr/bin/env python3
"""Extract <geoReference> from OpenDRIVE and emit Autoware map-origin YAML.

This module is intentionally self-contained: it does not depend on
crdesigner or commonroad, so it can be imported even when the conversion
pipeline is not loaded.

See docs/arch/map_origin.md (in the NEWSLabNTU fork) for the design
rationale and the proj4-parsing decision table.
"""

import re
from dataclasses import dataclass
from pathlib import Path

import yaml
from lxml import etree

# Autoware map-origin defaults (used when <geoReference> is missing/unparseable).
DEFAULT_PROJECTOR_TYPE = "TransverseMercator"
DEFAULT_VERTICAL_DATUM = "WGS84"
DEFAULT_SCALE_FACTOR = 0.9996       # UTM scale at the central meridian


@dataclass
class MapOrigin:
    """Map-origin record for the Autoware/TIER IV map_loader YAML schema."""

    projector_type: str = DEFAULT_PROJECTOR_TYPE
    vertical_datum: str = DEFAULT_VERTICAL_DATUM
    latitude: float = 0.0
    longitude: float = 0.0
    scale_factor: float = DEFAULT_SCALE_FACTOR
    # Diagnostic, not written to YAML:
    source: str = "default"          # "proj4" | "default" | "default-missing" | "default-invalid"
    proj_string: str | None = None


_PROJ_PARAM_RE = re.compile(r"\+(\w+)(?:=([^\s]+))?")


def _parse_proj4_params(proj_str: str) -> dict[str, str]:
    """Pull out `+key=value` pairs from a PROJ.4 string into a flat dict."""
    return {k: (v if v is not None else "") for k, v in _PROJ_PARAM_RE.findall(proj_str)}


def _utm_central_meridian(zone: int) -> float:
    return zone * 6 - 183


def _proj4_to_map_origin(proj_str: str) -> MapOrigin | None:
    """Map a PROJ.4 string to a MapOrigin, or None if unparseable.

    Handles three real-world cases seen in sample_data/:
      - CARLA bare `+lat_0=... +lon_0=...` (no `+proj=`)
      - `+proj=tmerc` with explicit lat_0/lon_0/k
      - `+proj=utm` with zone (and sometimes redundant lat_0/lon_0)
    Anything else is best-effort: keep lat_0/lon_0 if present, else None.
    """
    params = _parse_proj4_params(proj_str)
    proj = params.get("proj", "").lower()

    def _to_float(key: str) -> float | None:
        if key not in params or params[key] == "":
            return None
        try:
            return float(params[key])
        except ValueError:
            return None

    lat_0 = _to_float("lat_0")
    lon_0 = _to_float("lon_0")
    # PROJ.4 accepts both +k and +k_0 for scale.
    k = _to_float("k")
    if k is None:
        k = _to_float("k_0")
    datum = params.get("datum") or DEFAULT_VERTICAL_DATUM

    if proj == "utm":
        zone = None
        try:
            zone = int(params.get("zone", "")) if params.get("zone") else None
        except ValueError:
            zone = None
        zone_lon_0 = float(_utm_central_meridian(zone)) if zone is not None else None

        # Warn on inconsistent inputs: +proj=utm with explicit overrides that
        # disagree with the zone-derived canonical values. pyproj silently
        # ignores these overrides for +proj=utm, so the converted geometry will
        # not match what the proj4 string literally says (e.g. esmini's
        # e6mini.xodr has zone=32 (~9°E) but +lon_0=-122.086 (California)).
        if zone_lon_0 is not None and lon_0 is not None and abs(lon_0 - zone_lon_0) > 1e-6:
            print(
                f"[map-origin] WARNING: +proj=utm +zone={zone} implies "
                f"lon_0={zone_lon_0}, but the proj string sets +lon_0={lon_0}. "
                "pyproj will use the zone value; the converted .osm geometry "
                "may not match the YAML origin."
            )
        if lat_0 is not None and abs(lat_0) > 1e-6:
            print(
                f"[map-origin] WARNING: +proj=utm implies lat_0=0, but the "
                f"proj string sets +lat_0={lat_0}. pyproj will use 0."
            )
        if k is not None and abs(k - DEFAULT_SCALE_FACTOR) > 1e-6:
            print(
                f"[map-origin] WARNING: +proj=utm implies k=0.9996, but the "
                f"proj string sets +k/+k_0={k}. pyproj will use 0.9996."
            )

        # Trust the explicit values written in the proj string so the YAML
        # preserves the author's intent, even when pyproj ignores them. Fall
        # back to zone-derived values only when not supplied.
        if lon_0 is None:
            lon_0 = zone_lon_0
        if lat_0 is None:
            lat_0 = 0.0
        scale = k if k is not None else DEFAULT_SCALE_FACTOR
        return MapOrigin(
            projector_type="TransverseMercator",
            vertical_datum=datum,
            latitude=lat_0,
            longitude=lon_0 if lon_0 is not None else 0.0,
            scale_factor=scale,
            source="proj4",
            proj_string=proj_str,
        )

    if proj == "tmerc" or (proj == "" and (lat_0 is not None or lon_0 is not None)):
        # Bare CARLA form (no +proj=) lands here; default scale to 0.9996.
        scale = k if k is not None else (1.0 if proj == "tmerc" else DEFAULT_SCALE_FACTOR)
        return MapOrigin(
            projector_type="TransverseMercator",
            vertical_datum=datum,
            latitude=lat_0 if lat_0 is not None else 0.0,
            longitude=lon_0 if lon_0 is not None else 0.0,
            scale_factor=scale,
            source="proj4",
            proj_string=proj_str,
        )

    # Unknown projection: still emit a TransverseMercator origin from lat_0/lon_0
    # if available, so downstream tools have *something* to anchor on.
    if lat_0 is not None or lon_0 is not None:
        return MapOrigin(
            projector_type="TransverseMercator",
            vertical_datum=datum,
            latitude=lat_0 if lat_0 is not None else 0.0,
            longitude=lon_0 if lon_0 is not None else 0.0,
            scale_factor=k if k is not None else DEFAULT_SCALE_FACTOR,
            source="proj4",
            proj_string=proj_str,
        )

    return None


def extract_map_origin(xodr_path: str | Path) -> MapOrigin:
    """Read <geoReference> from an OpenDRIVE file and return its MapOrigin.

    Falls back to default values when the tag is missing or unparseable.
    The `source` field on the returned MapOrigin records which branch was taken
    so callers can warn appropriately.
    """
    try:
        with open(xodr_path, "rb") as f:
            tree = etree.parse(f)
    except Exception as e:
        print(f"[map-origin] Could not read {xodr_path}: {e}")
        return MapOrigin(source="default-missing")

    geo_elem = tree.find(".//geoReference")
    if geo_elem is None or not (geo_elem.text and geo_elem.text.strip()):
        return MapOrigin(source="default-missing")

    proj_str = geo_elem.text.strip()
    origin = _proj4_to_map_origin(proj_str)
    if origin is None:
        print(f"[map-origin] Unparseable <geoReference> in {xodr_path}: {proj_str!r}")
        return MapOrigin(source="default-invalid", proj_string=proj_str)
    return origin


def needs_proj_normalization(origin: MapOrigin) -> bool:
    """True if the parsed <geoReference> lacked a `+proj=` directive (e.g. CARLA).

    crdesigner's network.py calls `CRS(raw_geo_ref_string)` directly, and
    pyproj rejects PROJ.4 strings that omit `+proj=`. Such inputs must be
    rewritten before the file is handed to opendrive_to_commonroad.
    """
    return (
        origin.source == "proj4"
        and origin.proj_string is not None
        and "+proj=" not in origin.proj_string
    )


def normalized_proj4(origin: MapOrigin) -> str:
    """Construct a valid `+proj=tmerc` PROJ.4 string from a MapOrigin."""
    return (
        f"+proj=tmerc "
        f"+lat_0={origin.latitude} +lon_0={origin.longitude} "
        f"+k={origin.scale_factor} +x_0=0 +y_0=0 "
        f"+datum={origin.vertical_datum} +units=m +no_defs"
    )


def write_normalized_xodr(input_path: str | Path, output_path: str | Path, origin: MapOrigin) -> None:
    """Copy `input_path` to `output_path`, rewriting <geoReference> with a valid proj4 string."""
    tree = etree.parse(str(input_path))
    geo_elem = tree.find(".//geoReference")
    if geo_elem is not None:
        # etree.CDATA wraps the string so the output keeps the <![CDATA[...]]> form
        # that crdesigner's parser expects.
        for child in list(geo_elem):
            geo_elem.remove(child)
        geo_elem.text = etree.CDATA(normalized_proj4(origin))
    tree.write(str(output_path), xml_declaration=True, encoding="UTF-8")


def write_map_origin_yaml(path: str | Path, origin: MapOrigin) -> None:
    """Write a MapOrigin to a YAML file in the Autoware map_loader schema."""
    payload = {
        "projector_type": origin.projector_type,
        "vertical_datum": origin.vertical_datum,
        "map_origin": {
            "latitude": float(origin.latitude),
            "longitude": float(origin.longitude),
        },
        "scale_factor": float(origin.scale_factor),
    }
    with open(path, "w") as f:
        yaml.safe_dump(payload, f, sort_keys=False, default_flow_style=False)
