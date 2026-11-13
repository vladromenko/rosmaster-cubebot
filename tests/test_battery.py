from cubebot.battery import BatteryGauge, soc_from_rest_voltage


def test_curve_is_monotonic_and_bounded():
    values = [soc_from_rest_voltage(v / 100) for v in range(900, 1300, 5)]
    assert values[0] == 0 and values[-1] == 100
    assert all(b >= a for a, b in zip(values, values[1:]))


def test_gauge_filters_and_compensates_load():
    g = BatteryGauge(log_path=None)
    for i in range(30):
        g.update(12.0, now=float(i))
    rest = g.state()["soc"]
    g2 = BatteryGauge(log_path=None)
    for i in range(30):
        g2.update(11.85, base_moving=True, now=float(i))      # 0.15 V of sag under load
    assert abs(g2.state()["soc"] - rest) < 2.0


def test_outliers_do_not_jump_the_reading():
    g = BatteryGauge(log_path=None)
    for i in range(20):
        g.update(11.4, now=float(i))
    before = g.state()["soc"]
    g.update(9.0, now=21.0)
    assert abs(g.state()["soc"] - before) < 4.0


def test_remaining_time_from_discharge_and_charging_flag():
    g = BatteryGauge(log_path=None)
    for i in range(400):
        g.update(12.3 - i * 0.001, now=float(i))               # slow steady discharge
    assert g.remaining_minutes() is not None and g.remaining_minutes() > 0
    c = BatteryGauge(log_path=None)
    for i in range(40):
        c.update(12.7, now=float(i))
    assert c.charging and c.remaining_minutes() is None


def test_stale_reading_detection():
    g = BatteryGauge(log_path=None)
    g.update(12.0, now=0.0)
    g.update(12.0, now=30.0)
    assert g.stale(now=31.0, moving=True) and not g.stale(now=31.0, moving=False)
