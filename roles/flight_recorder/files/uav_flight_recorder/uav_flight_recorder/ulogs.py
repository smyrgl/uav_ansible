"""Fetch the FC's ulogs over MAVLink FTP and file them beside the bags.

PX4 writes /fs/microsd/log/<YYYY-MM-DD>/<HH_MM_SS>.ulg, named with the UTC
time at which logging started (arming, with SDLOG_MODE 0). The fetcher lists
the recent day folders, downloads every .ulg it has not fetched yet (a state
file remembers name and size), and links each one into the bag whose armed
interval starts within a few minutes of the log's start time, as fc.ulg, so the
offload pushes it with the bag and the replay host converts and aligns it.

Pure functions here (tested); the MAVLink FTP I/O in fetch_ulogs.py.
"""
import json
import os
import re
from datetime import datetime, timedelta, timezone

LOG_NAME = re.compile(r"^(\d{2})_(\d{2})_(\d{2})\.ulg$")
DAY_NAME = re.compile(r"^\d{4}-\d{2}-\d{2}$")


def log_start(day, name):
    """UTC datetime of /fs/microsd/log/<day>/<name>, or None."""
    m = LOG_NAME.match(name)
    if not m or not DAY_NAME.match(day):
        return None
    try:
        return datetime.strptime(day + " " + ":".join(m.groups()), "%Y-%m-%d %H:%M:%S").replace(tzinfo=timezone.utc)
    except ValueError:
        return None


def recent_days(days, today=None):
    """The last `days` day-folder names (UTC), newest first."""
    today = today or datetime.now(timezone.utc)
    return [(today - timedelta(days=i)).strftime("%Y-%m-%d") for i in range(days)]


def load_state(path):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def save_state(path, state):
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(state, f, indent=1)
    os.replace(tmp, path)


def wanted(entries, day, state):
    """[(name, size)] of the .ulg files in a day folder not fetched at this size yet."""
    out = []
    for name, size in entries:
        if not LOG_NAME.match(name):
            continue
        key = day + "/" + name
        if state.get(key, {}).get("size") == size and state.get(key, {}).get("fetched"):
            continue
        out.append((name, size))
    return out


def bag_for(start, bags, tolerance_s=300.0):
    """The bag dir whose armed_at is nearest to `start` within the tolerance.
    `bags` is {dir: armed_at_iso}; None when nothing is close enough."""
    best, best_dt = None, None
    for path, armed_at in bags.items():
        if not armed_at:
            continue
        try:
            armed = datetime.fromisoformat(str(armed_at).replace("Z", "+00:00"))
        except ValueError:
            continue
        if armed.tzinfo is None:
            armed = armed.replace(tzinfo=timezone.utc)
        dt = abs((armed - start).total_seconds())
        if dt <= tolerance_s and (best_dt is None or dt < best_dt):
            best, best_dt = path, dt
    return best


def bags_armed_at(flights_dir):
    """{bag dir: armed_at} for every bag with a flight.json."""
    out = {}
    try:
        names = os.listdir(flights_dir)
    except OSError:
        return out
    for name in names:
        path = os.path.join(flights_dir, name)
        meta = os.path.join(path, "flight.json")
        if not name.startswith("flight_") or not os.path.isfile(meta):
            continue
        try:
            with open(meta) as f:
                out[path] = json.load(f).get("armed_at")
        except (OSError, ValueError):
            continue
    return out
