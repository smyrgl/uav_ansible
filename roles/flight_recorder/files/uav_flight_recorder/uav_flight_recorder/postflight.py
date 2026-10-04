"""Post-flight checks for a closed bag, run by the recorder as a separate
low-priority process so the node never blocks:

- topic counts from rosbag2's metadata.yaml (or the MCAP summary when the
  recorder was killed before writing it), the required topics present, and
  the sign-off rule of the autonomy roadmap: zero messages on any /fmu/in topic;
- a gap scan of the key topics (message log times out of the MCAP index),
  over the armed interval when the bag has one, else the whole bag;
- the LIO map saved beside the bag (/lio/map/save, a std_srvs Trigger);
- everything merged into flight.json ("postflight", "signoff") plus a
  flight_card.md for the flight log.

Pure functions first; the CLI at the bottom glues them.
"""
import argparse
import glob
import json
import os
import shutil
import sys
import time
from datetime import datetime, timezone

NS = 1_000_000_000


def iso_to_ns(text):
    if not text:
        return None
    try:
        stamp = datetime.fromisoformat(str(text).replace("Z", "+00:00"))
    except ValueError:
        return None
    if stamp.tzinfo is None:
        stamp = stamp.replace(tzinfo=timezone.utc)
    return int(stamp.timestamp() * NS)


def counts_from_metadata(text):
    """{topic: message_count}, duration_ns, start_ns from a rosbag2 metadata.yaml."""
    import yaml
    info = (yaml.safe_load(text) or {}).get("rosbag2_bagfile_information", {})
    counts = {}
    for entry in info.get("topics_with_message_count", []) or []:
        name = (entry.get("topic_metadata") or {}).get("name")
        if name:
            counts[name] = int(entry.get("message_count", 0))
    duration = int((info.get("duration") or {}).get("nanoseconds", 0))
    start = int((info.get("starting_time") or {}).get("nanoseconds_since_epoch", 0))
    return counts, duration, start


def gap_stats(times_ns, window=None, max_gap_ms=100.0):
    """Intervals between consecutive log times, restricted to `window` =
    (start_ns, end_ns) when given. A gap is a missed message: an interval longer
    than the topic's nominal period (its median interval) plus `max_gap_ms`, so
    the same tolerance applies to a 100 Hz odometry and a 10 Hz lidar."""
    times = sorted(times_ns)
    if window and window[0] is not None and window[1] is not None:
        times = [t for t in times if window[0] <= t <= window[1]]
    result = {"count": len(times), "max_gap_ms": 0.0, "gaps_over": 0, "rate_hz": 0.0, "nominal_ms": None,
              "limit_ms": None, "first_ns": times[0] if times else None, "last_ns": times[-1] if times else None}
    if len(times) < 2:
        return result
    intervals = [b - a for a, b in zip(times, times[1:])]
    nominal = sorted(intervals)[len(intervals) // 2]
    limit = nominal + max_gap_ms * 1e6
    worst = max(intervals)
    result.update(max_gap_ms=round(worst / 1e6, 1), gaps_over=sum(1 for g in intervals if g > limit),
                  nominal_ms=round(nominal / 1e6, 1), limit_ms=round(limit / 1e6, 1),
                  rate_hz=round(len(intervals) * NS / (times[-1] - times[0]), 2))
    return result


def mcap_files(bag_path):
    return sorted(glob.glob(os.path.join(bag_path, "*.mcap")))


def scan(bag_path, topics, window=None, max_gap_ms=100.0, want_counts=False):
    """Message log times per topic out of the bag's MCAP files via the mcap
    library; `want_counts` also returns every channel's message count (for a bag
    without metadata.yaml). Raises ImportError without the library."""
    from mcap.reader import make_reader
    times = {t: [] for t in topics}
    counts = {}
    for path in mcap_files(bag_path):
        with open(path, "rb") as handle:
            reader = make_reader(handle)
            summary = reader.get_summary() if want_counts else None
            if summary is not None and summary.statistics is not None:
                for channel_id, count in summary.statistics.channel_message_counts.items():
                    channel = summary.channels.get(channel_id)
                    if channel is not None:
                        counts[channel.topic] = counts.get(channel.topic, 0) + int(count)
                selected = topics
            else:
                selected = None if want_counts else topics   # no index: one pass over everything
            for _schema, channel, message in reader.iter_messages(topics=selected):
                if channel.topic in times:
                    times[channel.topic].append(message.log_time)
                if want_counts and summary is None:
                    counts[channel.topic] = counts.get(channel.topic, 0) + 1
    gaps = {t: gap_stats(v, window, max_gap_ms) for t, v in times.items()}
    return gaps, counts


def signoff(counts, required, gaps, max_gap_ms):
    reasons = []
    fmu_in = sum(c for t, c in counts.items() if t.startswith("/fmu/in/"))
    if fmu_in:
        reasons.append("%d message(s) on /fmu/in topics" % fmu_in)
    for topic in required:
        if counts.get(topic, 0) == 0:
            reasons.append("no messages on %s" % topic)
    for topic, g in sorted(gaps.items()):
        if g["gaps_over"]:
            reasons.append("%d gap(s) over %.0f ms on %s (nominal %.0f ms, worst %.0f ms)" % (
                g["gaps_over"], g.get("limit_ms") or max_gap_ms, topic, g.get("nominal_ms") or 0, g["max_gap_ms"]))
    return {"pass": not reasons, "reasons": reasons, "fmu_in_messages": fmu_in}


def save_map(service, timeout_sec, bag_path):
    """Call the LIO map's Trigger service and copy the PCD it wrote next to the bag."""
    import rclpy
    from std_srvs.srv import Trigger
    rclpy.init()
    try:
        node = rclpy.create_node("uav_postflight_map_save")
        client = node.create_client(Trigger, service)
        if not client.wait_for_service(timeout_sec=min(10.0, timeout_sec)):
            return {"saved": False, "message": "service %s unavailable" % service}
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
        if not future.done():
            return {"saved": False, "message": "no response from %s within %.0f s" % (service, timeout_sec)}
        response = future.result()
        result = {"saved": bool(response.success), "message": response.message}
        if response.success and ": /" in response.message:
            source = response.message.rsplit(": ", 1)[-1].strip()
            if os.path.isfile(source):
                target = os.path.join(bag_path, "lio_map.pcd")
                shutil.copy2(source, target)
                result["pcd"] = target
        return result
    finally:
        rclpy.shutdown()


def flight_card(meta):
    post = meta.get("postflight", {})
    verdict = meta.get("signoff", {})
    lines = ["# Flight card: %s" % os.path.basename(meta.get("bag", "")), "",
             "| | |", "|---|---|",
             "| Started (UTC) | %s |" % meta.get("started_at"),
             "| Armed | %s |" % (meta.get("armed_at") or "never"),
             "| Disarmed | %s |" % (meta.get("disarmed_at") or "n/a"),
             "| Stopped | %s (%s) |" % (meta.get("stopped_at"), meta.get("stop_reason")),
             "| Duration | %s s, %.1f GB |" % (meta.get("duration_sec"), (meta.get("bytes") or 0) / 1e9),
             "| Recorder exit | %s |" % meta.get("recorder_exit_code"),
             "| Sign-off | %s |" % ("PASS" if verdict.get("pass") else "FAIL: " + "; ".join(verdict.get("reasons", []))),
             "| /fmu/in messages | %s |" % verdict.get("fmu_in_messages", "?"),
             "| LIO map | %s |" % (post.get("map", {}).get("pcd") or post.get("map", {}).get("message", "not saved")),
             "", "## Key topics", "", "| Topic | Messages | Rate Hz | Nominal ms | Worst gap ms | Gaps over limit |", "|---|---|---|---|---|---|"]
    for topic, g in sorted(post.get("gaps", {}).items()):
        lines.append("| %s | %d | %s | %s | %s | %d (limit %s) |" % (
            topic, g["count"], g["rate_hz"], g.get("nominal_ms"), g["max_gap_ms"], g["gaps_over"], g.get("limit_ms")))
    lines += ["", "Gap window: %s" % post.get("gap_window", "whole bag"),
              "", "## Fill in", "", "- Site / pattern / pilot:", "- Firmware (`ver all`) and parameter checksum:",
              "- Observations:", ""]
    return "\n".join(lines)


def run(bag_path, required, gap_topics, max_gap_ms, map_service, map_timeout, log=print):
    meta_path = os.path.join(bag_path, "flight.json")
    try:
        with open(meta_path) as handle:
            meta = json.load(handle)
    except (OSError, ValueError):
        meta = {"bag": bag_path}
    post = {"started_at": datetime.now(timezone.utc).isoformat(), "mcap_version": None}
    if map_service:
        try:
            post["map"] = save_map(map_service, map_timeout, bag_path)
        except Exception as exc:  # the map is a courtesy, never a failed flight
            post["map"] = {"saved": False, "message": "map save failed: %s" % exc}
        log("map: %s" % post["map"].get("message"))
    counts, duration_ns, start_ns = {}, 0, 0
    try:
        with open(os.path.join(bag_path, "metadata.yaml")) as handle:
            counts, duration_ns, start_ns = counts_from_metadata(handle.read())
        post["counts_source"] = "metadata.yaml"
    except OSError:
        post["counts_source"] = "mcap summary (metadata.yaml missing: the recorder did not close the bag)"
    window = (iso_to_ns(meta.get("armed_at")), iso_to_ns(meta.get("disarmed_at")))
    post["gap_window"] = ("armed %s to %s" % (meta.get("armed_at"), meta.get("disarmed_at"))
                          if window[0] and window[1] else "whole bag")
    gaps = {}
    try:
        import mcap
        post["mcap_version"] = getattr(mcap, "__version__", "?")
        gaps, scanned = scan(bag_path, gap_topics, window if window[0] and window[1] else None, max_gap_ms,
                             want_counts=not counts)
        if not counts:
            counts = scanned
    except ImportError:
        post["gap_scan"] = "skipped: the mcap library is not installed"
    except Exception as exc:
        post["gap_scan"] = "failed: %s" % exc
    post["gaps"] = gaps
    post["topics"] = len(counts)
    post["messages"] = sum(counts.values())
    post["required"] = {t: counts.get(t, 0) for t in required}
    post["duration_sec"] = round(duration_ns / NS, 1) if duration_ns else meta.get("duration_sec")
    post["finished_at"] = datetime.now(timezone.utc).isoformat()
    meta["postflight"] = post
    meta["signoff"] = signoff(counts, required, gaps, max_gap_ms)
    with open(meta_path, "w") as handle:
        json.dump(meta, handle, indent=2)
    with open(os.path.join(bag_path, "flight_card.md"), "w") as handle:
        handle.write(flight_card(meta))
    log("sign-off %s%s" % ("PASS" if meta["signoff"]["pass"] else "FAIL", "" if meta["signoff"]["pass"] else ": " + "; ".join(meta["signoff"]["reasons"])))
    return meta


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("bag")
    parser.add_argument("--required", nargs="*", default=[])
    parser.add_argument("--gap-topics", nargs="*", default=[])
    parser.add_argument("--max-gap-ms", type=float, default=100.0)
    parser.add_argument("--map-service", default="")
    parser.add_argument("--map-timeout", type=float, default=30.0)
    args = parser.parse_args(argv)
    try:
        os.nice(10)
    except OSError:
        pass
    started = time.monotonic()
    meta = run(args.bag, args.required, args.gap_topics, args.max_gap_ms, args.map_service or None, args.map_timeout)
    print("postflight done in %.0f s: %s" % (time.monotonic() - started, os.path.join(args.bag, "flight.json")))
    return 0 if meta["signoff"]["pass"] else 3


if __name__ == "__main__":
    sys.exit(main())
