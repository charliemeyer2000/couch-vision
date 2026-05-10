from __future__ import annotations

from random import Random

from .telemetry import (
    DistanceSource,
    Event,
    Sample,
    local_hostname,
    new_run_id,
    utc_now,
)


def simulate_samples(
    distances: list[float],
    hold_seconds: float,
    sample_rate_hz: float,
    seed: int = 42,
) -> list[Sample]:
    rng = Random(seed)
    run_id = new_run_id("sim")
    hostname = local_hostname()
    samples: list[Sample] = []
    sequence = 0
    elapsed = 0.0
    per_station = max(1, int(hold_seconds * sample_rate_hz))
    for distance in distances:
        sent = 0
        acked = 0
        timeouts = 0
        reconnects = 0
        drops = 0
        for _ in range(per_station):
            sequence += 1
            sent += 1
            base_latency = 22 + distance * 2.4 + rng.uniform(-5, 12)
            failure_probability = _failure_probability(distance)
            if rng.random() < failure_probability:
                timeouts += 1
                latency = None
                error = "simulated timeout"
            else:
                acked += 1
                latency = max(5.0, base_latency + rng.uniform(0, distance * 0.7))
                error = ""
            if distance >= 45 and rng.random() < 0.05:
                reconnects += 1
            if distance >= 55 and rng.random() < 0.10:
                drops += 1
            loss = (timeouts / sent) * 100
            samples.append(
                Sample(
                    run_id=run_id,
                    device_role="laptop_logger",
                    laptop_hostname=hostname,
                    orin_hostname="simulated-orin",
                    adapter_name="simulated-adapter",
                    adapter_address="",
                    controller_name="simulated-xbox-controller",
                    sequence=sequence,
                    timestamp=utc_now(),
                    elapsed_s=elapsed,
                    declared_distance_m=distance,
                    distance_source=DistanceSource.SIMULATION.value,
                    gps_latitude=None,
                    gps_longitude=None,
                    geolocation_accuracy_m=None,
                    rssi_dbm=-45 - distance * 0.9 + rng.uniform(-4, 4),
                    ping_rtt_ms=max(2.0, 8 + distance * 1.8 + rng.uniform(-2, 10)),
                    command_latency_ms=latency,
                    command_sent_count=sent,
                    command_ack_count=acked,
                    command_timeout_count=timeouts,
                    packet_loss_percent=loss,
                    reconnect_events=reconnects,
                    drop_events=drops,
                    error=error,
                    notes="simulation",
                )
            )
            elapsed += 1 / sample_rate_hz
    return samples


def simulate_events(
    samples: list[Sample],
    distances: list[float],
    hold_seconds: float,
) -> list[Event]:
    if not samples:
        return []
    run_id = samples[0].run_id
    events: list[Event] = []
    elapsed = 0.0
    event_index = 0
    for distance in distances:
        event_index += 1
        events.append(
            Event(
                record_type="event",
                event_id=f"event-{event_index:04d}",
                run_id=run_id,
                event_type="station_start",
                event_severity="info",
                timestamp=utc_now(),
                elapsed_s=elapsed,
                declared_distance_m=distance,
                distance_source=DistanceSource.SIMULATION.value,
                note=f"simulated station start at {distance:g} m",
            )
        )
        elapsed += hold_seconds
        event_index += 1
        events.append(
            Event(
                record_type="event",
                event_id=f"event-{event_index:04d}",
                run_id=run_id,
                event_type="station_end",
                event_severity="info",
                timestamp=utc_now(),
                elapsed_s=elapsed,
                declared_distance_m=distance,
                distance_source=DistanceSource.SIMULATION.value,
                note=f"simulated station end at {distance:g} m",
            )
        )
    return events


def _failure_probability(distance: float) -> float:
    match distance:
        case d if d < 25:
            return 0.002
        case d if d < 40:
            return 0.015
        case d if d < 55:
            return 0.08
        case _:
            return 0.22
