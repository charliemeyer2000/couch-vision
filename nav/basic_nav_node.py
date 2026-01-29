import os
import sys
import time
import json
import logging
import numpy as np
import requests
import googlemaps
import polyline as polyline_lib
from scipy.interpolate import splprep, splev
from pyproj import Transformer

import foxglove
from foxglove.schemas import (
    Timestamp,
    PointCloud,
    PackedElementField,
    PackedElementFieldNumericType,
    GeoJson,
    LocationFix,
)
from foxglove.websocket import (
    Capability,
    ChannelView,
    Client,
    ClientChannel,
    ServerListener,
)

API_KEY = os.environ.get("GOOGLE_MAPS_API_KEY")
ROTUNDA = "38.035853,-78.503307"

THE_FLATS = "38.032171,-78.493018"  # start point
POE_ROOM = "38.035623,-78.505148"  # end point


class ExampleListener(ServerListener):
    def __init__(self) -> None:
        # Map client id -> set of subscribed topics
        self.subscribers: dict[int, set[str]] = {}

    def has_subscribers(self) -> bool:
        return len(self.subscribers) > 0

    def on_subscribe(
        self,
        client: Client,
        channel: ChannelView,
    ) -> None:
        """
        Called by the server when a client subscribes to a channel.
        We'll use this and on_unsubscribe to simply track if we have any subscribers at all.
        """
        logging.info(f"Client {client} subscribed to channel {channel.topic}")
        self.subscribers.setdefault(client.id, set()).add(channel.topic)

    def on_unsubscribe(
        self,
        client: Client,
        channel: ChannelView,
    ) -> None:
        """
        Called by the server when a client unsubscribes from a channel.
        """
        logging.info(f"Client {client} unsubscribed from channel {channel.topic}")
        self.subscribers[client.id].remove(channel.topic)
        if not self.subscribers[client.id]:
            del self.subscribers[client.id]

    def on_client_advertise(
        self,
        client: Client,
        channel: ClientChannel,
    ) -> None:
        """
        Called when a client advertises a new channel.
        """
        logging.info(f"Client {client.id} advertised channel: {channel.id}")
        logging.info(f"  Topic: {channel.topic}")
        logging.info(f"  Encoding: {channel.encoding}")
        logging.info(f"  Schema name: {channel.schema_name}")
        logging.info(f"  Schema encoding: {channel.schema_encoding}")
        logging.info(f"  Schema: {channel.schema!r}")

    def on_message_data(
        self,
        client: Client,
        client_channel_id: int,
        data: bytes,
    ) -> None:
        """
        This handler demonstrates receiving messages from the client.
        You can send messages from Foxglove app in the publish panel:
        https://docs.foxglove.dev/docs/visualization/panels/publish
        """
        logging.info(f"Message from client {client.id} on channel {client_channel_id}")
        logging.info(f"Data: {data!r}")

    def on_client_unadvertise(
        self,
        client: Client,
        client_channel_id: int,
    ) -> None:
        """
        Called when a client unadvertises a new channel.
        """
        logging.info(f"Client {client.id} unadvertised channel: {client_channel_id}")


class BasicNavNode:
    def __init__(self):
        self.path_points = None
        self.gps_points = None
        self.origin_lat, self.origin_lon = map(float, ROTUNDA.split(","))

        if not API_KEY:
            raise ValueError("GOOGLE_MAPS_API_KEY environment variable not set.")

        self.gmaps = googlemaps.Client(key=API_KEY)

        # Get Start and End Points
        origin, destination = self.get_user_input()

        # Generate Path
        self.path_points = self.generate_path(origin, destination)

        if self.path_points is None:
            print("Failed to generate path. Exiting.")
            sys.exit(1)

    def get_user_input(self):
        print(f"Default Start (The Flats): {THE_FLATS}")
        # Use simple non-blocking input or default for now to avoid hanging if run headless
        # For this example, we'll just check if we have command line args or default
        if len(sys.argv) > 1:
            return THE_FLATS, POE_ROOM  # Simplify for automation if needed

        # In a real continuous integration agent scenario, input() might hang.
        # But per user instructions, we just assume the code remains similar or interactive.
        # I'll keep the logic but wrap it to be safe or just use defaults if not interactive.
        try:
            start_input = input(
                "Enter start 'lat,lon' (or press Enter for default): "
            ).strip()
        except EOFError:
            start_input = ""

        origin = start_input if start_input else THE_FLATS

        print(f"Default End (Poe Room): {POE_ROOM}")
        try:
            end_input = input(
                "Enter end 'lat,lon' (or press Enter for default): "
            ).strip()
        except EOFError:
            end_input = ""

        destination = end_input if end_input else POE_ROOM

        print(f"Origin: {origin}")
        print(f"Destination: {destination}")
        return origin, destination

    def snap_to_roads(self, points_latlon):
        snapped = []
        chunk_size = 100
        for i in range(0, len(points_latlon), chunk_size):
            chunk = points_latlon[i : i + chunk_size]
            path = "|".join([f"{lat},{lon}" for lat, lon in chunk])
            url = "https://roads.googleapis.com/v1/snapToRoads"
            try:
                r = requests.get(
                    url,
                    params={"path": path, "interpolate": "true", "key": API_KEY},
                    timeout=20,
                )
                r.raise_for_status()
                data = r.json()
                for p in data.get("snappedPoints", []):
                    loc = p["location"]
                    snapped.append((loc["latitude"], loc["longitude"]))
            except Exception as e:
                print(f"Failed to snap to roads: {e}")
        return snapped

    def resample_path(self, xy_points, step_size=0.5):
        if len(xy_points) < 2:
            return xy_points

        diffs = np.diff(xy_points, axis=0)
        dists = np.linalg.norm(diffs, axis=1)
        cumulative_dist = np.concatenate(([0], np.cumsum(dists)))
        total_dist = cumulative_dist[-1]

        num_points = int(total_dist / step_size) + 1
        new_dists = np.linspace(0, total_dist, num_points)

        try:
            k = 3 if len(xy_points) > 3 else 1
            tck, u = splprep(xy_points.T, u=cumulative_dist, s=0, k=k)
            new_points = splev(new_dists, tck)
            return np.column_stack(new_points)
        except Exception as e:
            print(f"Spline interpolation failed: {e}, falling back to linear")
            return xy_points

    def generate_path(self, origin, destination):
        # 1. Get Directions
        try:
            directions = self.gmaps.directions(origin, destination, mode="driving")
            if not directions:
                print("No directions found.")
                return None
            enc = directions[0]["overview_polyline"]["points"]
            latlon = polyline_lib.decode(enc)
        except Exception as e:
            print(f"Google Maps Direction API failed: {e}")
            return None

        print(f"Original points: {len(latlon)}")

        # 2. Snap to Roads
        latlon_snapped = self.snap_to_roads(latlon)
        print(f"Snapped points: {len(latlon_snapped)}")

        if not latlon_snapped:
            print(" snapping returned no points, using original.")
            latlon_snapped = latlon

        self.gps_points = latlon_snapped

        # 3. Project to ENU (Rotunda Origin)
        lat_0, lon_0 = map(float, ROTUNDA.split(","))
        proj_string = f"+proj=tmerc +lat_0={lat_0} +lon_0={lon_0} +k=1 +x_0=0 +y_0=0 +ellps=WGS84 +datum=WGS84 +units=m +no_defs"
        tfm = Transformer.from_crs("EPSG:4326", proj_string, always_xy=True)

        xy = np.array(
            [tfm.transform(lon, lat) for lat, lon in latlon_snapped], dtype=float
        )

        print("Projected to ENU.")

        # 4. Resample
        xy_resampled = self.resample_path(xy, step_size=0.5)
        print(f"Resampled to {len(xy_resampled)} points.")

        return xy_resampled


def main():
    foxglove.set_log_level(logging.DEBUG)

    node = BasicNavNode()

    if node.path_points is None:
        return

    # Prepare PointCloud data once since it's static for this run
    # Format: x (float32), y (float32), z (float32)
    # 3 floats * 4 bytes/float = 12 bytes per point
    # stride = 12

    points_array = np.zeros((len(node.path_points), 3), dtype=np.float32)
    points_array[:, 0] = node.path_points[:, 0]
    points_array[:, 1] = node.path_points[:, 1]
    points_array[:, 2] = 0.0  # Z is 0

    data_bytes = points_array.tobytes()

    # Prepare GeoJSON for the path
    geojson_data = {
        "type": "FeatureCollection",
        "features": [
            {
                "type": "Feature",
                "properties": {},
                "geometry": {
                    "type": "LineString",
                    "coordinates": [
                        [lon, lat] for lat, lon in node.gps_points
                    ],  # GeoJSON is [lon, lat]
                },
            }
        ],
    }
    geojson_str = json.dumps(geojson_data)

    listener = ExampleListener()

    server = foxglove.start_server(
        server_listener=listener,
        capabilities=[Capability.ClientPublish],
        supported_encodings=["json"],
    )

    print("Server started on ws://localhost:8765")

    try:
        while True:
            ts = Timestamp.from_datetime(datetime.datetime.now())

            # Publish Origin
            foxglove.log(
                "/location",
                LocationFix(
                    latitude=node.origin_lat,
                    longitude=node.origin_lon,
                    altitude=0,
                    frame_id="world",
                ),
            )

            # Publish GPS Path (GeoJSON) - send occasionally or once?
            # Foxglove usually handles periodic updates fine.
            foxglove.log("/gps_path", GeoJson(geojson=geojson_str))

            # Publish PointCloud
            foxglove.log(
                "/path_points",
                PointCloud(
                    frame_id="world",
                    timestamp=ts,
                    point_stride=12,
                    fields=[
                        PackedElementField(
                            name="x",
                            offset=0,
                            type=PackedElementFieldNumericType.Float32,
                        ),
                        PackedElementField(
                            name="y",
                            offset=4,
                            type=PackedElementFieldNumericType.Float32,
                        ),
                        PackedElementField(
                            name="z",
                            offset=8,
                            type=PackedElementFieldNumericType.Float32,
                        ),
                    ],
                    data=data_bytes,
                ),
            )

            time.sleep(0.1)

    except KeyboardInterrupt:
        server.stop()


if __name__ == "__main__":
    import datetime  # Ensure datetime is imported for Timestamp

    main()
