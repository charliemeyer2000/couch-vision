import os
from datetime import datetime, timezone
from urllib.parse import unquote_plus

import boto3

INDEX_KEY = "index.txt"


def handler(event, context):
    # Skip events that only touch index.txt. Writing the index must not
    # re-invoke this function (S3 ObjectCreated on put_object).
    if _is_index_only_event(event):
        return {"statusCode": 200, "files": 0, "skipped": True}

    bucket = os.environ["BUCKET_NAME"]
    s3 = boto3.client("s3")

    objects = []
    paginator = s3.get_paginator("list_objects_v2")
    for page in paginator.paginate(Bucket=bucket):
        for obj in page.get("Contents", []):
            key = obj["Key"]
            if key == INDEX_KEY:
                continue
            objects.append(obj)

    lines = [
        "couch-vision-bags file listing "
        f"(updated {datetime.now(timezone.utc).strftime('%Y-%m-%d %H:%M UTC')})",
        "",
    ]
    total_size = 0
    for obj in sorted(objects, key=lambda o: o["Key"]):
        size = obj["Size"]
        total_size += size
        modified = obj["LastModified"].strftime("%Y-%m-%d %H:%M")
        lines.append(f"{modified}  {_fmt_size(size):>10s}  {obj['Key']}")

    lines.append("")
    lines.append(f"Total: {len(objects)} files, {_fmt_size(total_size)}")
    lines.append("")

    s3.put_object(
        Bucket=bucket,
        Key=INDEX_KEY,
        Body="\n".join(lines),
        ContentType="text/plain",
    )
    return {"statusCode": 200, "files": len(objects)}


def _fmt_size(n):
    for unit in ("B", "KiB", "MiB", "GiB", "TiB"):
        if n < 1024:
            return f"{n:.1f} {unit}"
        n /= 1024
    return f"{n:.1f} PiB"


def _event_object_keys(event):
    """Extract object keys from classic S3 notifications or EventBridge events."""
    if not isinstance(event, dict):
        return []

    keys = []
    for record in event.get("Records") or []:
        key = record.get("s3", {}).get("object", {}).get("key")
        if key:
            keys.append(unquote_plus(key))
    if keys:
        return keys

    detail = event.get("detail") or {}
    key = detail.get("object", {}).get("key")
    if key:
        return [unquote_plus(key)]
    return []


def _is_index_only_event(event):
    """True when every referenced key is index.txt.

    Manual/test invokes with no object keys still refresh the listing.
    """
    keys = _event_object_keys(event)
    return bool(keys) and all(key == INDEX_KEY for key in keys)
