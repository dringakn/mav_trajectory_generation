#!/usr/bin/env python3
"""
test_waypoints_processor.py

Demonstrates all WaypointsProcessor features:
  - Heading modes: auto, manual, fixed, poi
  - Coordinate systems: gps, enu, ned, ecef
  - Path planning: goto_waypoint(s), goto_height, takeoff, land, abort
  - Shapes: create_rectangle, create_circle, create_figure_eight
  - Distance & bounds checking
  - Interpolation on/off
  - Odometry progression & segment tracking
"""
import os
import math
from waypoints_processor import WaypointsProcessor, Waypoint

# Wurzburg, Germany
# Reference point: 49.7939° N, 9.9512° E, 120 m altitude
REF_LAT = 49.7939
REF_LON = 9.9512
REF_ALT = 10.0
TAKEOFF_HEIGHT = 10.0
LANDING_HEIGHT = 0.0
INTERPOLATE = True
INTERPOLATE_DISTANCE = 5.0

def print_wp_list(label, wps):
    print(f"\n{label}:")
    for i, wp in enumerate(wps):
        yaw_deg = math.degrees(wp.yaw) if wp.yaw is not None else float('nan')
        print(f"  [{i}] x={wp.x:.2f}, y={wp.y:.2f}, z={wp.z:.2f}, yaw°={yaw_deg:.2f}")

def print_gps_list(label, gps_list):
    print(f"\n{label} (lat, lon, alt, yaw°):")
    for i, (lat, lon, alt, yaw) in enumerate(gps_list):
        yaw_deg = math.degrees(yaw) if yaw is not None else float('nan')
        print(f"  [{i}] {lat:.6f}, {lon:.6f}, {alt:.2f}, {yaw_deg:.2f}")

def demo_heading_modes():
    for mode in ("auto", "manual", "fixed", "poi"):
        print(f"\n=== HEADING MODE: {mode.upper()} ===")
        params = {
            "heading_mode": mode,
            "interpolate_waypoints": INTERPOLATE,
            "intermediate_waypoint_distance": INTERPOLATE_DISTANCE,
            "takeoff_height": TAKEOFF_HEIGHT,
            "landing_height": LANDING_HEIGHT,
        }
        proc = WaypointsProcessor(params)

        # 1) Reference & odometry
        proc.set_reference(REF_LAT, REF_LON, REF_ALT)       # GPS origin
        proc.set_odometry(0.0, 0.0, 0.0, yaw=0.0)    # ENU origin

        # 2) Mode-specific setup
        if mode == "poi":
            proc.set_point_of_interest((REF_LAT+0.0001, REF_LON+0.0001, REF_ALT), "gps")
        if mode == "fixed":
            proc.set_fixed_angle(45.0)  # degrees

        # 3) Plan a multi-point GPS path
        raw_gps = [
            Waypoint(REF_LAT+0.00005, REF_LON+0.00005, REF_ALT+10.0, yaw=math.pi/4),
            Waypoint(REF_LAT+0.0001,  REF_LON+0.0001,  REF_ALT+20.0, yaw=math.pi/2),
        ]
        path = proc.goto_waypoints(raw_gps, liftoff=True, coord="gps")
        print_wp_list(" Local ENU path", path)
        print_gps_list(" GPS output", proc.get_waypoints_gps())

        # 4) Single-waypoint in ENU
        target_enu = Waypoint(50.0, 50.0, 15.0)
        single = proc.goto_waypoint(target_enu, coord="enu")
        print_wp_list(" Single ENU target", single)

        # 5) Vertical moves
        print_wp_list(" Go to 25 m", proc.goto_height(25.0))
        print_wp_list(" Takeoff",       proc.takeoff())
        print_wp_list(" Land",          proc.land())

        # 6) Shapes in ENU
        print_wp_list(" Rectangle",     proc.create_rectangle(Waypoint(0,0,20), 20, 10, "enu"))
        print_wp_list(" Circle",        proc.create_circle(Waypoint(0,0,20), 15,    "enu"))
        print_wp_list(" Figure-8",      proc.create_figure_eight(Waypoint(0,0,20), 10, "enu"))

        # 7) Distance & bounds
        d = proc.get_distance_between(path[0], path[-1], "enu")
        inside = proc.check_within_bounds(
            path,
            Waypoint(-100,-100,-10),
            Waypoint(100,100,100),
            "enu"
        )
        print(f"\n Distance first→last: {d:.2f} m")
        print(f" All within ±100 m bounds? {inside}")

        # 8) Simulate odometry progression
        print("\n Simulating progression:")
        for wp in path[:: max(1, len(path)//4) ]:
            proc.set_odometry(wp.x, wp.y, wp.z, wp.yaw)
            print(f"  segment → {proc.get_current_segment()}")

        # 9) Abort
        proc.abort()
        print(f"\n After abort, waypoints: {len(proc.get_waypoints_local())}")

def demo_coordinate_modes(heading="auto"):
    print(f"\n=== COORDINATE MODES DEMO ({heading} heading) ===")
    params = {
        "heading_mode": heading,
        "interpolate_waypoints": INTERPOLATE,
        "intermediate_waypoint_distance": INTERPOLATE_DISTANCE,
        "takeoff_height": TAKEOFF_HEIGHT,
        "landing_height": LANDING_HEIGHT,
    }
    proc = WaypointsProcessor(params)
    proc.set_reference(REF_LAT, REF_LON, REF_ALT)
    proc.set_odometry(0.0, 0.0, 0.0, 0.0)

    # NED input: (north, east, down)
    raw_ned = [Waypoint(100.0, 50.0, -20.0), Waypoint(150.0, 75.0, -30.0)]
    path_ned = proc.goto_waypoints(raw_ned, liftoff=False, coord="ned")
    print_wp_list(" Path from NED input", path_ned)
    print_gps_list("  → GPS output", proc.get_waypoints_gps())

    # ECEF input: convert one GPS point to ECEF coords
    lat, lon, alt = REF_LAT+0.0002, REF_LON+0.0002, 25.0
    xe, ye, ze = proc.gc.geodetic2ecef(lat, lon, alt + proc.reference_altitude)
    raw_ecef = [Waypoint(xe, ye, ze)]
    path_ecef = proc.goto_waypoints(raw_ecef, liftoff=False, coord="ecef")
    print_wp_list(" Path from ECEF input", path_ecef)
    print_gps_list("  → GPS output", proc.get_waypoints_gps())

def demo_export_shapes():
    print("\n=== EXPORTING ALL SHAPES TO GeoJSON ===")
    params = {
        "heading_mode": "auto",
        "interpolate_waypoints": INTERPOLATE,
        "intermediate_waypoint_distance": INTERPOLATE_DISTANCE,
        "takeoff_height": TAKEOFF_HEIGHT,
        "landing_height": LANDING_HEIGHT,
    }
    proc = WaypointsProcessor(params)
    proc.set_reference(REF_LAT, REF_LON, REF_ALT)
    proc.set_odometry(0.0, 0.0, 20.0, yaw=0.0)
    
    center = Waypoint(0, 0, 20)

    shapes = {
        "rectangle":       list(proc.create_rectangle(center, 30, 15, "enu")),
        "circle":          list(proc.create_circle(center, 20,    "enu")),
        "ellipse":         list(proc.create_ellipse(center, 25, 15, "enu")),
        "figure8":         list(proc.create_figure_eight(center, 15, "enu")),
        "spiral":          list(proc.create_spiral(center, r_max=100, turns=3, coord="enu")),
        "helix":           list(proc.create_helix(center, radius=10, height=30, turns=2, coord="enu")),
        "lawnmower":       list(proc.create_lawnmower(center, width=40, height=20, rows=5, coord="enu")),
        "lissajous":       list(proc.create_lissajous(center, A=15, B=10, a=3, b=2, delta=math.pi/4, coord="enu")),
        "lemniscate":      list(proc.create_lemniscate(center, r=12, coord="enu")),
        "cardioid":        list(proc.create_cardioid(center, r=12, coord="enu")),
        "star":            list(proc.create_star(center, radius=15, points=5, skip=2, coord="enu")),
        "rosette":         list(proc.create_rosette(center, R=12, alpha=4, k=6, coord="enu")),
        "expanding_sq":    list(proc.create_expanding_square(center, step=5, loops=3, coord="enu")),
        "sector_scan":     list(proc.create_sector_scan(center, radius=20, start_b=0, end_b=90, revolutions=2, coord="enu")),
        "diag_zigzag":     list(proc.create_diagonal_zigzag(center, length=40, width=20, spacing=5, coord="enu")),
        "hilbert":         list(proc.create_hilbert(center, size=30, order=2, coord="enu")),
    }

    out_dir = "exported_shapes"
    os.makedirs(out_dir, exist_ok=True)

    for name, wps in shapes.items():
        proc.waypoints = wps
        fname = os.path.join(out_dir, f"{name}.geojson")
        proc.export_waypoints_to_geojson(fname)
        print(f"  • {name:15s} → {fname} has {len(wps)} waypoints")

if __name__ == "__main__":
    demo_heading_modes()
    demo_coordinate_modes(heading="auto")
    demo_export_shapes()
    