#!/usr/bin/env python3
"""Generate the shakedown flight card as a QGroundControl mission (.plan).

The card's manoeuvres become mission items PX4 flies on its own under
supervision, identically every flight: takeoff, timed hovers at 3/10/20 m,
full turns in place at 5 and 20 m as eight held headings, legs out
and back at 2 and 5 m/s (and 8 m/s with --fast), a figure-8, a pass over
featureless ground, a low pass over ground mapped earlier, a small survey
grid, then a landing at home. The FC's MISSION_CURRENT stream marks every item
in the bag's flight.json (flight recorder), so no marker switch is needed.

    make_mission.py --home <lat> <lon> --home-alt <m AMSL> \
        --heading 0 --leg 30 -o docs/flight-cards/<date>-<name>.plan

Review it in QGC (drag the featureless and mapped-ground passes onto the right
ground, check the legs fit the yard) before uploading. PX4: multicopter
waypoints with a hold time are flown to the point; NAV_ACC_RAD applies to the
rest. Altitudes are relative to home.
"""
import argparse
import json
import math

NAV_WAYPOINT, NAV_LAND, NAV_TAKEOFF, DO_CHANGE_SPEED = 16, 21, 22, 178


class Plan:
    def __init__(self, lat, lon, alt_amsl, heading_deg):
        self.lat, self.lon, self.alt = lat, lon, alt_amsl
        self.heading = math.radians(heading_deg)
        self.items = []

    def offset(self, forward_m, right_m):
        """lat/lon of a point `forward_m` along the heading and `right_m` to its right."""
        north = forward_m * math.cos(self.heading) - right_m * math.sin(self.heading)
        east = forward_m * math.sin(self.heading) + right_m * math.cos(self.heading)
        lat = self.lat + north / 111_320.0
        lon = self.lon + east / (111_320.0 * math.cos(math.radians(self.lat)))
        return round(lat, 8), round(lon, 8)

    def add(self, command, params, frame=3, altitude=None, altitude_mode=1):
        self.items.append({"AMSLAltAboveTerrain": None, "Altitude": altitude, "AltitudeMode": altitude_mode,
                           "autoContinue": True, "command": command, "doJumpId": len(self.items) + 1,
                           "frame": frame, "params": params, "type": "SimpleItem"})

    def waypoint(self, forward, right, alt, hold=0.0, yaw=None, radius=1.0):
        lat, lon = self.offset(forward, right)
        self.add(NAV_WAYPOINT, [hold, radius, 0, yaw, lat, lon, alt], altitude=alt)

    def speed(self, mps):
        self.add(DO_CHANGE_SPEED, [1, mps, -1, 0, 0, 0, 0], frame=2, altitude=0, altitude_mode=0)

    def yaw_sweep(self, alt, steps=8, hold=6.0):
        """A full turn in place as waypoints at the same spot with the heading in
        the waypoint's yaw field (PX4 does not execute the condition-yaw command
        in missions; QGC shows it as "Waiting For Yaw", unsupported). Static
        holds at several headings are what the antenna lever arm needs anyway."""
        base = math.degrees(self.heading)
        for k in range(steps + 1):
            self.waypoint(0, 0, alt, hold=hold, yaw=round((base + 360.0 * k / steps) % 360.0, 1))

    def takeoff(self, alt):
        self.add(NAV_TAKEOFF, [0, 0, 0, None, self.lat, self.lon, alt], altitude=alt)

    def land_home(self):
        self.add(NAV_LAND, [0, 0, 0, None, self.lat, self.lon, 0], altitude=0)

    def document(self, cruise=5.0, hover=2.0):
        return {"fileType": "Plan", "version": 1, "groundStation": "QGroundControl",
                "geoFence": {"circles": [], "polygons": [], "version": 2},
                "rallyPoints": {"points": [], "version": 2},
                "mission": {"version": 2, "firmwareType": 12, "vehicleType": 2, "globalPlanAltitudeMode": 1,
                            "cruiseSpeed": cruise, "hoverSpeed": hover,
                            "plannedHomePosition": [self.lat, self.lon, self.alt], "items": self.items}}


def shakedown(plan, leg, fast=False, hover_s=30.0, fig8_r=10.0, survey_spacing=10.0):
    manifest = []
    def mark(label):
        manifest.append((len(plan.items) + 1, label))
    mark("takeoff to 3 m"); plan.takeoff(3)
    mark("hover 3 m"); plan.waypoint(0, 0, 3, hold=hover_s)
    mark("hover 10 m"); plan.waypoint(0, 0, 10, hold=hover_s)
    mark("descend to 5 m"); plan.waypoint(0, 0, 5, hold=5)
    mark("turn in place at 5 m: 8 held headings"); plan.yaw_sweep(5)
    mark("hover 20 m"); plan.waypoint(0, 0, 20, hold=hover_s)
    mark("turn in place at 20 m: 8 held headings"); plan.yaw_sweep(20)
    for speed in ([2.0, 5.0] + ([8.0] if fast else [])):
        mark("leg out and back at %.0f m/s, 20 m" % speed); plan.speed(speed)
        plan.waypoint(leg, 0, 20, hold=3, radius=2.0)
        plan.waypoint(0, 0, 20, hold=3, radius=2.0)
    mark("figure-8 at 15 m, 5 m/s"); plan.speed(5.0)
    for k in range(13):                       # a lemniscate across the heading
        t = 2 * math.pi * k / 12
        f = fig8_r * 2 * math.sin(t) / (1 + math.cos(t) ** 2)
        r = fig8_r * 2 * math.sin(t) * math.cos(t) / (1 + math.cos(t) ** 2)
        plan.waypoint(f, r, 15, hold=0, radius=2.0)
    plan.waypoint(0, 0, 15, hold=3)
    mark("featureless pass at 10 m, 3 m/s (move onto flat ground in QGC)"); plan.speed(3.0)
    plan.waypoint(0, -leg / 2, 10, hold=2, radius=2.0)
    plan.waypoint(0, leg / 2, 10, hold=2, radius=2.0)
    mark("low pass over mapped ground at 5 m, 3 m/s"); plan.waypoint(leg, 0, 5, hold=2, radius=2.0)
    plan.waypoint(0, 0, 5, hold=2, radius=2.0)
    mark("survey grid at 20 m, 5 m/s"); plan.speed(5.0)
    lines = 3
    for i in range(lines):
        right = (i - (lines - 1) / 2) * survey_spacing
        a, b = (0, leg) if i % 2 == 0 else (leg, 0)
        plan.waypoint(a, right, 20, hold=0, radius=2.0)
        plan.waypoint(b, right, 20, hold=0, radius=2.0)
    mark("return and land at home"); plan.speed(3.0); plan.waypoint(0, 0, 20, hold=3); plan.land_home()
    return manifest


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--home", nargs=2, type=float, required=True, metavar=("LAT", "LON"))
    ap.add_argument("--home-alt", type=float, default=0.0, help="home altitude AMSL (QGC replaces it at upload)")
    ap.add_argument("--heading", type=float, default=0.0, help="direction of the legs, degrees from north")
    ap.add_argument("--leg", type=float, default=30.0, help="leg length in metres")
    ap.add_argument("--fast", action="store_true", help="add the 8 m/s leg (needs a field)")
    ap.add_argument("--hover", type=float, default=30.0, help="hover hold seconds")
    ap.add_argument("-o", "--output", required=True)
    ap.add_argument("--manifest", help="write the item-to-manoeuvre table here (markdown)")
    args = ap.parse_args(argv)
    plan = Plan(args.home[0], args.home[1], args.home_alt, args.heading)
    manifest = shakedown(plan, args.leg, args.fast, args.hover)
    with open(args.output, "w") as f:
        json.dump(plan.document(), f, indent=2)
    lines = ["| Item | Manoeuvre |", "|---|---|"] + ["| %d | %s |" % (i, label) for i, label in manifest]
    if args.manifest:
        with open(args.manifest, "w") as f:
            f.write("\n".join(lines) + "\n")
    print("%d mission items -> %s" % (len(plan.items), args.output))
    print("\n".join(lines))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
