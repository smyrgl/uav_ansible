#!/usr/bin/env python3
"""ulog -> MCAP with UTC timestamps (autonomy roadmap, Stage 0b).

The PX4 ulog stamps everything in hrt microseconds since FC boot. The XRCE
timesync (TimesyncStatus, source_protocol 2) gives the offset the FC keeps
between its clock and the companion's UTC: estimated_offset = px4 - companion.
So UTC_us = hrt_us - estimated_offset, interpolated between timesync samples.
Two independent cross-checks come out of the same log:

- pps_capture: the hrt time of each PPS edge and the receiver's UTC for it
  (rtc_timestamp, an exact second). utc(hrt) - rtc per edge is the alignment
  error of the conversion itself; the Stage 0b gate wants its median under 2 ms.
- vehicle_gps_position.time_utc_usec: the fix epoch's UTC against the sample's
  hrt; loose (receiver latency), but it catches a whole-second or sign error.

Every dataset becomes a channel /ulog/<name>[_<instance>] with JSON messages
(one per row, every field), log_time = publish_time = UTC ns. Parameters and
logged text go to /ulog/parameters and /ulog/logged_messages. The conversion
summary (offset source, residual statistics) is an MCAP metadata record
"ulog2mcap" and, with --summary, a JSON file.

    ulog2mcap.py flight.ulg -o flight_ulog.mcap [--offset timesync|pps|none]
                 [--topics vehicle_odometry,vehicle_local_position_setpoint] [--summary s.json]

Pure functions first (tested without pyulog/mcap); the I/O at the bottom.
"""
import argparse
import bisect
import json
import math
import os
import sys
import time

US = 1_000_000
NS_PER_US = 1_000


def interpolate(series, x):
    """Piecewise-linear value of a sorted [(x, y), ...] series at x, held flat
    beyond the ends. Pure Python so the tests need no numpy."""
    if not series:
        return None
    xs = [p[0] for p in series]
    i = bisect.bisect_left(xs, x)
    if i <= 0:
        return series[0][1]
    if i >= len(series):
        return series[-1][1]
    (x0, y0), (x1, y1) = series[i - 1], series[i]
    if x1 == x0:
        return y1
    return y0 + (y1 - y0) * (x - x0) / (x1 - x0)


def timesync_series(stamps, offsets, protocols=None, dds_only=True):
    """[(hrt_us, estimated_offset_us)] from timesync_status rows, DDS samples
    only by default (source_protocol 2); sorted and de-duplicated."""
    rows = []
    for i, (t, o) in enumerate(zip(stamps, offsets)):
        if dds_only and protocols is not None and int(protocols[i]) != 2:
            continue
        rows.append((int(t), int(o)))
    rows.sort()
    out = []
    for t, o in rows:
        if out and out[-1][0] == t:
            out[-1] = (t, o)
        else:
            out.append((t, o))
    return out


def pps_series(stamps, rtc):
    """[(hrt_us, offset_us)] from pps_capture rows: offset = hrt - rtc, the same
    sign convention as timesync (px4 - utc). Rows without an rtc are skipped."""
    out = [(int(t), int(t) - int(r)) for t, r in zip(stamps, rtc) if int(r) > 0]
    out.sort()
    return out


def to_utc_us(series, hrt_us):
    offset = interpolate(series, hrt_us)
    return None if offset is None else int(round(hrt_us - offset))


def residual_stats(residuals_us):
    """count, median, p95 and worst |residual| in microseconds, plus the signed
    median (positive: the converted time runs ahead of the reference)."""
    signed = sorted(float(r) for r in residuals_us)
    values = sorted(abs(r) for r in signed)
    if not values:
        return {"count": 0, "median_us": None, "p95_us": None, "max_us": None, "signed_median_us": None}
    def pct(p):
        k = min(len(values) - 1, max(0, int(math.ceil(p * len(values)) - 1)))
        return values[k]
    return {"count": len(values), "median_us": pct(0.5), "p95_us": pct(0.95), "max_us": values[-1],
            "signed_median_us": signed[len(signed) // 2]}


def cross_check(series, stamps, reference_utc_us, valid=None):
    """Residuals utc(hrt) - reference for rows where valid (if given) is true."""
    residuals = []
    for i, (t, ref) in enumerate(zip(stamps, reference_utc_us)):
        if valid is not None and not valid[i]:
            continue
        if int(ref) <= 0:
            continue
        utc = to_utc_us(series, int(t))
        if utc is not None:
            residuals.append(utc - int(ref))
    return residual_stats(residuals)


def json_schema(field_names, field_types):
    """A jsonschema object for a dataset: integers, numbers, booleans; arrays are
    already flattened by pyulog into name[i] fields."""
    props = {}
    for name, ftype in zip(field_names, field_types):
        if ftype in ("bool",):
            props[name] = {"type": "boolean"}
        elif ftype in ("float", "double"):
            props[name] = {"type": "number"}
        elif ftype == "char":
            props[name] = {"type": "string"}
        else:
            props[name] = {"type": "integer"}
    return {"type": "object", "properties": props}


def plain(value):
    """numpy scalars and NaN/inf to JSON-safe Python values."""
    if hasattr(value, "item"):
        value = value.item()
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, bytes):
        return value.decode("ascii", "replace")
    return value


# --- I/O --------------------------------------------------------------------

def load_series(ulog, source):
    """The offset series and a description, from the chosen source."""
    def dataset(name):
        return [d for d in ulog.data_list if d.name == name]
    if source in ("timesync", "auto"):
        for d in dataset("timesync_status"):
            series = timesync_series(d.data["timestamp"], d.data["estimated_offset"], d.data.get("source_protocol"))
            if len(series) >= 2:
                return series, "timesync_status.estimated_offset (%d DDS samples)" % len(series)
        if source == "timesync":
            return None, "no DDS timesync_status samples in this log"
    if source in ("pps", "auto"):
        for d in dataset("pps_capture"):
            series = pps_series(d.data["timestamp"], d.data["rtc_timestamp"])
            if len(series) >= 2:
                return series, "pps_capture hrt - rtc_timestamp (%d edges)" % len(series)
        if source == "pps":
            return None, "no pps_capture edges with an rtc_timestamp in this log"
    return None, "none: timestamps stay hrt microseconds since boot (log_time = hrt ns)"


def checks(ulog, series):
    out = {}
    if series is None:
        return out
    for d in ulog.data_list:
        if d.name == "pps_capture":
            out["pps_capture"] = cross_check(series, d.data["timestamp"], d.data["rtc_timestamp"])
        elif d.name == "vehicle_gps_position" and "time_utc_usec" in d.data:
            fix = d.data.get("fix_type")
            valid = None if fix is None else [int(f) >= 3 for f in fix]
            out["vehicle_gps_position_%d" % d.multi_id] = cross_check(series, d.data["timestamp"], d.data["time_utc_usec"], valid)
    return out


def convert(ulog_path, out_path, offset_source="auto", topics=None, log=print):
    from pyulog import ULog
    from mcap.writer import Writer
    try:
        from mcap.writer import CompressionType
        compression = CompressionType.ZSTD
        import zstandard  # noqa: F401  (ZSTD needs it)
    except Exception:
        compression = None
    started = time.monotonic()
    ulog = ULog(ulog_path)
    series, source_text = load_series(ulog, offset_source)
    summary = {"ulog": os.path.basename(ulog_path), "offset_source": source_text,
               "offset_samples": len(series) if series else 0, "cross_checks": checks(ulog, series),
               "start_hrt_us": int(ulog.start_timestamp), "last_hrt_us": int(ulog.last_timestamp),
               "datasets": 0, "messages": 0, "channels": {}}
    if series:
        summary["start_utc_us"] = to_utc_us(series, int(ulog.start_timestamp))
        summary["last_utc_us"] = to_utc_us(series, int(ulog.last_timestamp))
    log("offset source: %s" % source_text)
    for name, stats in summary["cross_checks"].items():
        log("cross-check %s: n=%d median %s us, p95 %s us, max %s us" % (
            name, stats["count"], stats["median_us"], stats["p95_us"], stats["max_us"]))
    wanted = set(topics) if topics else None
    with open(out_path, "wb") as handle:
        kwargs = {"compression": compression} if compression is not None else {}
        writer = Writer(handle, **kwargs)
        writer.start(profile="", library="uav_ansible ulog2mcap")
        for d in sorted(ulog.data_list, key=lambda x: (x.name, x.multi_id)):
            if wanted is not None and d.name not in wanted:
                continue
            names = [f.field_name for f in d.field_data]
            types = [f.type_str for f in d.field_data]
            topic = "/ulog/%s" % d.name + ("_%d" % d.multi_id if d.multi_id else "")
            schema_id = writer.register_schema(name="ulog/%s" % d.name, encoding="jsonschema",
                                               data=json.dumps(json_schema(names, types)).encode())
            channel_id = writer.register_channel(topic=topic, message_encoding="json", schema_id=schema_id)
            stamps = d.data["timestamp"]
            columns = {n: d.data[n] for n in names if n in d.data}
            count = 0
            for i in range(len(stamps)):
                hrt = int(stamps[i])
                utc_us = to_utc_us(series, hrt) if series else hrt
                row = {n: plain(col[i]) for n, col in columns.items()}
                row["utc_us"] = utc_us
                writer.add_message(channel_id=channel_id, log_time=utc_us * NS_PER_US, publish_time=utc_us * NS_PER_US,
                                   sequence=i, data=json.dumps(row).encode())
                count += 1
            summary["channels"][topic] = count
            summary["datasets"] += 1
            summary["messages"] += count
        # parameters and logged text
        schema_id = writer.register_schema(name="ulog/parameters", encoding="jsonschema",
                                           data=json.dumps({"type": "object"}).encode())
        channel_id = writer.register_channel(topic="/ulog/parameters", message_encoding="json", schema_id=schema_id)
        t0 = int(ulog.start_timestamp)
        utc0 = (to_utc_us(series, t0) if series else t0) * NS_PER_US
        writer.add_message(channel_id=channel_id, log_time=utc0, publish_time=utc0, sequence=0,
                           data=json.dumps({k: plain(v) for k, v in ulog.initial_parameters.items()}).encode())
        schema_id = writer.register_schema(name="ulog/logged_message", encoding="jsonschema", data=json.dumps(
            {"type": "object", "properties": {"log_level": {"type": "integer"}, "message": {"type": "string"}}}).encode())
        channel_id = writer.register_channel(topic="/ulog/logged_messages", message_encoding="json", schema_id=schema_id)
        for i, m in enumerate(ulog.logged_messages):
            hrt = int(m.timestamp)
            utc_us = to_utc_us(series, hrt) if series else hrt
            writer.add_message(channel_id=channel_id, log_time=utc_us * NS_PER_US, publish_time=utc_us * NS_PER_US,
                               sequence=i, data=json.dumps({"log_level": int(m.log_level), "message": m.message,
                                                            "utc_us": utc_us}).encode())
        summary["seconds"] = round(time.monotonic() - started, 1)
        writer.add_metadata(name="ulog2mcap", data={k: json.dumps(v) if not isinstance(v, str) else v
                                                    for k, v in summary.items()})
        writer.finish()
    log("wrote %s: %d datasets, %d messages in %.1f s" % (out_path, summary["datasets"], summary["messages"], summary["seconds"]))
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("ulog")
    parser.add_argument("-o", "--output", help="MCAP path (default: <ulog>.mcap)")
    parser.add_argument("--offset", choices=("auto", "timesync", "pps", "none"), default="auto")
    parser.add_argument("--topics", help="comma-separated dataset names to keep (default: all)")
    parser.add_argument("--summary", help="write the conversion summary JSON here")
    args = parser.parse_args(argv)
    out = args.output or os.path.splitext(args.ulog)[0] + ".mcap"
    topics = [t.strip() for t in args.topics.split(",")] if args.topics else None
    summary = convert(args.ulog, out, args.offset, topics)
    if args.summary:
        with open(args.summary, "w") as handle:
            json.dump(summary, handle, indent=2)
    pps = summary["cross_checks"].get("pps_capture", {})
    if summary["offset_samples"] and pps.get("count") and pps["median_us"] is not None and pps["median_us"] > 2000:
        print("ALIGNMENT OUTSIDE THE 2 ms GATE: pps median %.0f us" % pps["median_us"])
        return 3
    return 0


if __name__ == "__main__":
    sys.exit(main())
