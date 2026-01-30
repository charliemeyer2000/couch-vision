import sys
from mcap.reader import make_reader
from mcap.well_known import MessageEncoding

def inspect_bag(path):
    print(f"Inspecting: {path}")
    with open(path, "rb") as f:
        reader = make_reader(f)
        print("Topics:")
        for schema in reader.get_summary().schemas.values():
            print(f"  Schema: {schema.name} ({schema.encoding})")

        for channel in reader.get_summary().channels.values():
            print(f"  Topic: {channel.topic} (Schema: {reader.get_summary().schemas[channel.schema_id].name})")

        print("\nChecking first few messages of potential interest:")
        # We can't decode easily without the schema definitions if they are complex ROS2 messages,
        # but mcap might have them embedded.
        # Let's just list topics for now.

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python inspect_mcap.py <path_to_mcap>")
        sys.exit(1)
    inspect_bag(sys.argv[1])
