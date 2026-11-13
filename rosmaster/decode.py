"""Telemetry decoders copied from the Explorer host (controller_feedback.py), unchanged in behaviour."""
import math
import struct


def decode_status(payload):
    if len(payload) != 216:
        raise ValueError('incompatible status layout')
    times = struct.unpack_from('<6Q', payload)
    mode, fault, arm_enabled, arm_cancel = payload[48:52]
    wheels = []
    for i, name in enumerate(('front_left', 'front_right', 'rear_left', 'rear_right')):
        raw, delta, target, measured, pwm, integral, motor = struct.unpack_from('<Hi4fI', payload, 52+26*i)
        if motor != (1, 3, 2, 4)[i] or not all(math.isfinite(v) for v in (target, measured, pwm, integral)):
            raise ValueError('bad wheel telemetry')
        wheels.append(dict(name=name, motor=motor, raw_counter=raw, raw_delta=delta,
                           target_rad_s=target, measured_rad_s=measured, pwm=pwm, integral=integral))
    velocity = struct.unpack_from('<3f', payload, 156)
    validity = struct.unpack_from('<I', payload, 212)[0]
    if validity not in (0, 1):
        raise ValueError('incompatible measurement validity flags')
    if not all(math.isfinite(v) for v in velocity):
        raise ValueError('bad measured base velocity')
    names = ('control_max_us', 'command_drops', 'result_drops', 'telemetry_drops', 'rejected_commands')
    return dict(acquired_us=times[0], boot=times[1], session=times[2], sequence=times[3],
                expires_us=times[4], finite_end_us=times[5], mode=mode, fault=fault,
                arm_enabled=bool(arm_enabled), arm_cancel_pending=bool(arm_cancel),
                wheels=wheels, velocity=list(velocity),
                encoder_measurement_valid=bool(validity),
                diagnostics=dict(zip(names, struct.unpack_from('<5I', payload, 168)),
                    host_rx_overruns=struct.unpack_from('<I', payload, 204)[0],
                    host_uart_errors=struct.unpack_from('<I', payload, 208)[0]),
                arm_sent_generation=struct.unpack_from('<Q', payload, 188)[0],
                highest_session=struct.unpack_from('<Q', payload, 196)[0])




class ScanAssembler:
    def __init__(self):
        self.ranges = [math.nan]*666
        self.intensities = [0.]*666
        self.started_us = None
        self.last_us = None
        self.covered = set()

    def packet(self, payload):
        if len(payload) < 25:
            raise ValueError('lidar packet truncated')
        stamp, errors = struct.unpack_from('<QI', payload)
        data = payload[12:]
        header, ct, count, first, last, checksum = struct.unpack_from('<HBBHHH', data)
        if header != 0x55aa or not count or len(data) != 10 + 3*count or not (first & 1 and last & 1):
            raise ValueError('lidar packet header/angles invalid')
        check = header ^ (ct | count << 8) ^ first ^ last
        for i in range(count):
            intensity, distance = struct.unpack_from('<BH', data, 10+3*i)
            check ^= intensity ^ distance
        if check != checksum:
            raise ValueError('lidar checksum invalid')
        complete = None
        if ct & 1:
            if self.started_us is not None and self.last_us is not None and 50000 <= stamp-self.started_us <= 400000:
                complete = dict(acquired_us=self.started_us, ended_us=stamp, ranges=self.ranges,
                                intensities=self.intensities, covered=len(self.covered), packet_errors=errors)
            self.ranges, self.intensities, self.covered = [math.nan]*666, [0.]*666, set()
            self.started_us = stamp
        if self.last_us is not None and not 0 < stamp-self.last_us < 100000:
            self.started_us = None
        self.last_us = stamp
        start_deg, end_deg = (first >> 1)/64., (last >> 1)/64.
        difference = (end_deg-start_deg) % 360
        for i in range(count):
            intensity, raw = struct.unpack_from('<BH', data, 10+3*i)
            distance = (raw >> 2) / 1000.
            angle = (start_deg + (difference*i/(count-1) if count > 1 else 0)) % 360
            index = round(angle*666/360) % 666
            self.ranges[index] = distance if .05 <= distance <= 12 else math.nan
            self.intensities[index] = float(intensity)
            self.covered.add(index)
        return complete
