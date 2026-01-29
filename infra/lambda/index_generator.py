import os
import boto3
from datetime import datetime, timezone


def handler(event, context):
    bucket = os.environ["BUCKET_NAME"]
    s3 = boto3.client("s3")

    objects = []
    paginator = s3.get_paginator("list_objects_v2")
    for page in paginator.paginate(Bucket=bucket):
        for obj in page.get("Contents", []):
            key = obj["Key"]
            if key == "index.txt":
                continue
            objects.append(obj)

    lines = [f"couch-vision-bags file listing (updated {datetime.now(timezone.utc).strftime('%Y-%m-%d %H:%M UTC')})", ""]
    total_size = 0
    for obj in sorted(objects, key=lambda o: o["Key"]):
        size = obj["Size"]
        total_size += size
        modified = obj["LastModified"].strftime("%Y-%m-%d %H:%M")
        lines.append(f"{modified}  {_fmt_size(size):>10s}  {obj['Key']}")

    lines.append("")
    lines.append(f"Total: {len(objects)} files, {_fmt_size(total_size)}")
    lines.append("")

    s3.put_object(Bucket=bucket, Key="index.txt", Body="\n".join(lines), ContentType="text/plain")
    return {"statusCode": 200, "files": len(objects)}


def _fmt_size(n):
    for unit in ("B", "KiB", "MiB", "GiB", "TiB"):
        if n < 1024:
            return f"{n:.1f} {unit}"
        n /= 1024
