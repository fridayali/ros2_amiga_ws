#!/usr/bin/env python3
import argparse
import asyncio
from pathlib import Path
from math import radians, cos, pi

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler

from farm_ng.canbus.canbus_pb2 import Twist2d
from farm_ng.core.event_client import EventClient
from farm_ng.core.event_service_pb2 import EventServiceConfig, EventServiceConfigList, SubscribeRequest
from farm_ng.core.events_file_reader import proto_from_json_file
from farm_ng.gps import gps_pb2

# Sabit datum (anchor) noktası. Aracın kendi filter servisi sanal/değişken
# bir anchor seçip pose'u o anchor'a göre basıyor; bu anchor araç ilerlerken
# kayabiliyor ve /rtk/odom'da sıçramalara yol açıyor. Bunun önüne geçmek için
# pozisyonu, aracın anchor'undan tamamen bağımsız olarak, burada sabitlenen
# tek bir referans noktasına (datum) göre WGS84 lat/lon'dan hesaplıyoruz.
DEFAULT_ANCHOR_LAT_DEG = 39.796011
DEFAULT_ANCHOR_LON_DEG = 32.531534
EARTH_RADIUS_M = 6378137.0  # WGS84 ekvator yarıçapı, yerel düzlem projeksiyonu için yeterli


def latlon_to_local_xy(lat_deg: float, lon_deg: float, anchor_lat_deg: float, anchor_lon_deg: float) -> tuple[float, float]:
    """WGS84 lat/lon'u sabit anchor'a göre yerel ENU (x=doğu, y=kuzey) metreye çevirir.

    Equirectangular (flat-earth) yaklaşımı; çiftlik ölçeğindeki (birkaç km) mesafeler için yeterli hassasiyette.
    """
    anchor_lat_rad = radians(anchor_lat_deg)
    dlat = radians(lat_deg - anchor_lat_deg)
    dlon = radians(lon_deg - anchor_lon_deg)
    x_east = dlon * cos(anchor_lat_rad) * EARTH_RADIUS_M
    y_north = dlat * EARTH_RADIUS_M
    return x_east, y_north


def filter_heading_to_enu_yaw(heading_rad: float) -> float:
    """Farm-ng filter servisinin north-referanslı heading'ini (0=kuzey) ENU/ROS
    yaw'a (0=doğu/+x, 90°=kuzey/+y) çevirir.

    Gerçek sürüş testiyle ölçüldü: araç ileri sürüldüğünde gerçek hareket
    yönü (x,y'den hesaplanan) ile filter heading'i arasında sabit ~90°
    fark vardı (ENU yaw = heading - 90°). Bu offset tahmini değil, ölçülmüş.
    """
    return heading_rad - pi / 2


class MultiClientSubscriber(Node):
    """Example of subscribing to events from multiple clients and publishing to ROS2."""

    def __init__(
        self,
        service_config: EventServiceConfigList,
        anchor_lat_deg: float = DEFAULT_ANCHOR_LAT_DEG,
        anchor_lon_deg: float = DEFAULT_ANCHOR_LON_DEG,
    ) -> None:
        super().__init__("multi_client_subscriber_ros")

        self.service_config = service_config
        self._clients: dict[str, EventClient] = {}   # <-- Değiştirildi
        self._subscriptions = []                     # <-- Değiştirildi
        self.orientation: float = None

        # Sabit datum (anchor) noktası — tüm /rtk/odom yayınları bu tek noktaya göre.
        self.anchor_lat_deg = anchor_lat_deg
        self.anchor_lon_deg = anchor_lon_deg
        self.get_logger().info(
            f"RTK odom datum (sabit anchor): lat={self.anchor_lat_deg:.6f}, lon={self.anchor_lon_deg:.6f}"
        )

        # ROS2 publishers
        self.pub_fix  = self.create_publisher(NavSatFix, "/gps/fix", 10)
        self.pub_pose = self.create_publisher(PoseStamped, "/gps/pose", 10)
        self.pub_odom = self.create_publisher(Odometry, "/rtk/odom", 10)

        # GPS cache
        self.last_lat = 0.0
        self.last_lon = 0.0
        self.last_alt = 0.0
        self.last_yaw = 0.0
        self.last_fix_ok = False

        # Populate the event clients
        config: EventServiceConfig
        for config in self.service_config.configs:
            if not config.port:
                self._subscriptions = config.subscriptions  # <-- Değiştirildi
                continue
            self._clients[config.name] = EventClient(config)

    async def _subscribe(self, subscription: SubscribeRequest) -> None:
        client_name: str = subscription.uri.query.split("=")[-1]
        client: EventClient = self._clients[client_name]

        # NOT: client.subscribe() stream'i ara sıra (CPU yükü, ağ vb.) hata vermeden
        # sessizce sonlanabiliyor. Bu durumda async for döngüsü biter ve eskiden bu
        # subscription bir daha asla yenilenmiyordu (sessiz, kalıcı veri kesintisi).
        # Bu yüzden dışına bir retry döngüsü eklendi.
        while True:
            try:
                async for event, message in client.subscribe(subscription, decode=True):
                    await self._handle_message(client_name, event, message)
            except Exception as exc:
                self.get_logger().error(f"[{client_name}] subscribe hatası: {exc}")
            self.get_logger().warn(f"[{client_name}] subscription kapandı, yeniden bağlanılıyor...")
            await asyncio.sleep(1.0)

    async def _handle_message(self, client_name: str, event, message) -> None:
        if client_name == "gps" and isinstance(message, gps_pb2.GpsFrame):
            self.last_lat = message.latitude
            self.last_lon = message.longitude
            self.last_alt = message.altitude
            # Eğer GPS verisinde headingMotion yoksa, filter orientation'ı kullan
            gps_yaw = getattr(message, "headingMotion", None)
            if gps_yaw is not None and gps_yaw != 0.0:
                self.last_yaw = gps_yaw
            elif self.orientation is not None:
                self.last_yaw = self.orientation  # filter'dan gelen yaw
            fix_ok = getattr(message, "gnss_fix_ok", True)
            self.last_fix_ok = fix_ok

            # NavSatFix publish
            gps_msg = NavSatFix()
            gps_msg.header.stamp = self.get_clock().now().to_msg()
            gps_msg.header.frame_id = "gps_link"
            gps_msg.latitude = self.last_lat
            gps_msg.longitude = self.last_lon
            gps_msg.altitude = self.last_alt
            gps_msg.status.status = (
                NavSatStatus.STATUS_FIX if fix_ok else NavSatStatus.STATUS_NO_FIX
            )
            gps_msg.status.service = NavSatStatus.SERVICE_GPS
            gps_msg.position_covariance = [
                4.0, 0.0, 0.0,
                0.0, 4.0, 0.0,
                0.0, 0.0, 9.0
            ]
            gps_msg.position_covariance_type = 2
            self.pub_fix.publish(gps_msg)

            # PoseStamped publish
            pose_msg = PoseStamped()
            pose_msg.header = gps_msg.header
            pose_msg.pose.position.x = self.last_lat
            pose_msg.pose.position.y = self.last_lon
            pose_msg.pose.position.z = self.last_alt
            q = quaternion_from_euler(0, 0, radians(self.last_yaw))
            pose_msg.pose.orientation.x = q[0]
            pose_msg.pose.orientation.y = q[1]
            pose_msg.pose.orientation.z = q[2]
            pose_msg.pose.orientation.w = q[3]
            self.pub_pose.publish(pose_msg)

            print(f"[GPS] {self.last_lat:.6f}, {self.last_lon:.6f}, {self.last_alt:.2f}, yaw={self.last_yaw:.2f}")

        elif client_name == "filter":
            # NOT: message.pose, aracın filter servisinin kendi seçtiği
            # değişken/sanal anchor'a göredir — araç ilerlerken bu anchor
            # kayabiliyor ve pozisyonda sıçrama yaratıyor. Bu yüzden pose.translation
            # KULLANILMIYOR. Pozisyon, aşağıda sabit datum'a göre GPS lat/lon'dan
            # hesaplanıyor. heading ise anchor'dan bağımsız, mutlak (absolute) bir
            # büyüklük olduğu için filter'dan alınıyor — ama filter'ın heading'i
            # north-referanslı (0=kuzey); ENU/ROS yaw'a (0=doğu) çevriliyor.
            if hasattr(message, "heading"):
                self.orientation = filter_heading_to_enu_yaw(message.heading)

            if self.orientation is None:
                return

            if not self.last_fix_ok or (self.last_lat == 0.0 and self.last_lon == 0.0):
                self.get_logger().warn(
                    "/rtk/odom yayınlanamadı: geçerli bir GPS fix yok.", throttle_duration_sec=5.0
                )
                return

            # Sabit anchor'a göre yerel ENU pozisyon (anchor kayması burada etkisiz)
            x_local, y_local = latlon_to_local_xy(
                self.last_lat, self.last_lon, self.anchor_lat_deg, self.anchor_lon_deg
            )

            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = "map"
            odom_msg.child_frame_id = "base_link"

            # Pozisyon (sabit datum'a göre Kartezyen koordinatlar)
            odom_msg.pose.pose.position.x = x_local
            odom_msg.pose.pose.position.y = y_local
            odom_msg.pose.pose.position.z = 0.0

            # Yönelim (mutlak heading -> quaternion)
            q = quaternion_from_euler(0, 0, self.orientation)
            odom_msg.pose.pose.orientation.x = q[0]
            odom_msg.pose.pose.orientation.y = q[1]
            odom_msg.pose.pose.orientation.z = q[2]
            odom_msg.pose.pose.orientation.w = q[3]
            odom_msg.pose.covariance = [
                0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.05, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.1
            ]

            self.pub_odom.publish(odom_msg)

            print(
                f"[GPS+ANCHOR→ODOM] x={x_local:.3f}, "
                f"y={y_local:.3f}, "
                f"yaw={self.orientation:.3f}"
            )

    async def run(self) -> None:
        tasks: list[asyncio.Task] = []
        for subscription in self._subscriptions:   # <-- Değiştirildi
            tasks.append(asyncio.create_task(self._subscribe(subscription)))
        await asyncio.gather(*tasks)


async def main_async(config_path: Path, anchor_lat_deg: float, anchor_lon_deg: float):
    service_config = proto_from_json_file(config_path, EventServiceConfigList())
    node = MultiClientSubscriber(service_config, anchor_lat_deg, anchor_lon_deg)
    await node.run()


def main():
    parser = argparse.ArgumentParser(description="Farm-ng GPS/Filter ROS2 bridge")
    parser.add_argument("--config", type=Path, required=True, help="Path to config.json")
    parser.add_argument(
        "--anchor-lat", type=float, default=DEFAULT_ANCHOR_LAT_DEG,
        help="Sabit RTK datum enlemi (derece). Varsayılan: %(default)s",
    )
    parser.add_argument(
        "--anchor-lon", type=float, default=DEFAULT_ANCHOR_LON_DEG,
        help="Sabit RTK datum boylamı (derece). Varsayılan: %(default)s",
    )
    args = parser.parse_args()

    rclpy.init()

    try:
        asyncio.run(main_async(args.config, args.anchor_lat, args.anchor_lon))
    except KeyboardInterrupt:
        print("Stopped by user.")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
