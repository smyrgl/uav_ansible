import json

from uav_camera.clock import ClockMapping, unwrap_near

WRAP = (1 << 32) * 1000
MODEL = {"valid": True, "reason": "ok", "wrap_ns": WRAP, "device_ref_ns": 1_582_268_303_000,
         "offset_ref_ns": 1_790_913_255_823_259_392, "skew_ppm": -11.8, "computed_utc_ns": 1_790_914_838_581_424_905}


def test_maps_a_stamp_with_offset_and_skew():
    clock = ClockMapping()
    clock.update(json.dumps(MODEL))
    device = MODEL["device_ref_ns"] + 2_000_000_000
    expected = device + MODEL["offset_ref_ns"] + round(-11.8e-6 * 2_000_000_000)
    assert clock.to_utc_ns(device, now_ns=MODEL["computed_utc_ns"] + 1) == expected


def test_invalid_stale_or_missing_models_map_nothing():
    clock = ClockMapping(max_age_s=5.0)
    assert clock.to_utc_ns(MODEL["device_ref_ns"]) is None
    clock.update(json.dumps({**MODEL, "valid": False, "reason": "collecting: 3 of 10 one-second bins"}))
    assert clock.to_utc_ns(MODEL["device_ref_ns"]) is None and "collecting" in clock.reason
    clock.update(json.dumps(MODEL))
    assert clock.to_utc_ns(MODEL["device_ref_ns"], now_ns=MODEL["computed_utc_ns"] + 6_000_000_000) is None
    assert clock.reason == "clock model stale"
    clock.update("not json")
    assert clock.model is None


def test_wrapped_stamps_map_continuously():
    model = {**MODEL, "device_ref_ns": WRAP - 1_000_000_000}
    clock = ClockMapping()
    clock.update(json.dumps(model))
    before = clock.to_utc_ns(WRAP - 500_000_000, now_ns=model["computed_utc_ns"])
    after = clock.to_utc_ns(500_000_000, now_ns=model["computed_utc_ns"])   # raw counter after the rollover
    # one second of device time, scaled by the model's -11.8 ppm skew
    assert after - before == 1_000_000_000 + round(-11.8e-6 * 1_000_000_000)
    assert unwrap_near(5, WRAP + 3, WRAP) == WRAP + 5
