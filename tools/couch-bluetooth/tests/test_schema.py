from couch_range.telemetry import CSV_FIELDS, Sample, sample_to_row


def test_sample_schema_has_required_fields() -> None:
    required = {
        "run_id",
        "declared_distance_m",
        "distance_source",
        "command_latency_ms",
        "command_ack_count",
        "packet_loss_percent",
    }
    assert required <= set(CSV_FIELDS)


def test_sample_to_row() -> None:
    sample = Sample(
        run_id="r",
        device_role="laptop_logger",
        laptop_hostname="laptop",
        orin_hostname="orin",
        adapter_name="adapter",
        adapter_address="",
        controller_name="controller",
        sequence=1,
        timestamp="now",
        elapsed_s=0.1,
        declared_distance_m=5,
        distance_source="manual_marker",
        gps_latitude=None,
        gps_longitude=None,
        geolocation_accuracy_m=None,
        rssi_dbm=None,
        ping_rtt_ms=None,
        command_latency_ms=10,
        command_sent_count=1,
        command_ack_count=1,
        command_timeout_count=0,
        packet_loss_percent=0,
        reconnect_events=0,
        drop_events=0,
        error="",
        notes="",
    )
    assert sample_to_row(sample)["sequence"] == 1
