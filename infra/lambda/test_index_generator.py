from index_generator import _event_object_keys, _is_index_only_event


def test_skip_classic_s3_index_created():
    event = {
        "Records": [
            {
                "eventName": "ObjectCreated:Put",
                "s3": {"object": {"key": "index.txt"}},
            }
        ]
    }
    assert _is_index_only_event(event) is True


def test_skip_classic_s3_index_deleted():
    event = {
        "Records": [
            {
                "eventName": "ObjectRemoved:Delete",
                "s3": {"object": {"key": "index.txt"}},
            }
        ]
    }
    assert _is_index_only_event(event) is True


def test_process_bag_upload():
    event = {
        "Records": [
            {
                "eventName": "ObjectCreated:Put",
                "s3": {"object": {"key": "drive.mcap"}},
            }
        ]
    }
    assert _is_index_only_event(event) is False


def test_process_mixed_batch():
    event = {
        "Records": [
            {"s3": {"object": {"key": "index.txt"}}},
            {"s3": {"object": {"key": "other.mcap"}}},
        ]
    }
    assert _is_index_only_event(event) is False


def test_process_manual_invoke():
    assert _is_index_only_event({}) is False
    assert _is_index_only_event({"Records": []}) is False


def test_skip_eventbridge_index_key():
    event = {
        "source": "aws.s3",
        "detail-type": "Object Created",
        "detail": {"object": {"key": "index.txt"}},
    }
    assert _event_object_keys(event) == ["index.txt"]
    assert _is_index_only_event(event) is True


def test_process_eventbridge_bag_key():
    event = {
        "source": "aws.s3",
        "detail-type": "Object Created",
        "detail": {"object": {"key": "bags/run-1.mcap"}},
    }
    assert _is_index_only_event(event) is False


def test_url_encoded_key():
    event = {
        "Records": [
            {"s3": {"object": {"key": "path%2Fwith%20space.mcap"}}},
        ]
    }
    assert _event_object_keys(event) == ["path/with space.mcap"]
