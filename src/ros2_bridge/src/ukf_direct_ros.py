#!/usr/bin/env python3
"""Gercek farm_ng UKF'sini (UkFilterWrapper + RobotFilterSe2) tek bir process'te
calistirir, GPS/IMU/CAN verisini gRPC/protobuf-over-network yerine DOGRUDAN
donanimdan okur, ve sonucu roslibpy ile mevcut rosbridge_server (container
icinde, port 9090) uzerinden ROS2 topic'lerine publish eder.

Bu, gps_filter.py + farm-ng'nin kendi gps/imu/filter servislerinin YERINE
GECMEZ -- onlarla AYNI ANDA, AYNI donanimi (GPS seri port, OAK IMU) acmaya
calisirsa cihaz cakismasi olur. Bu yuzden test ederken farm-ng'nin gps+imu
servisleri durdurulmus olmali (canbus/filter servislerine dokunulmadi).

Sonuc, MEVCUT /gps/fix, /gps/pose, /rtk/odom topic'lerinin YERINE degil,
AYRI /ukf_direct/* topic'lerine basiliyor -- boylece nav2/EKF gibi mevcut
hicbir tuketici etkilenmiyor, sadece karsilastirma icin yeni bir veri akisi
eklendi.

ROBOT (Jetson) UZERINDE, HOST'ta /farm_ng_image/venv ile calistirilmali
(container icinde DEGIL -- depthai/ukf_wrapper bu venv'e ozel):
    /farm_ng_image/venv/bin/python3 ukf_direct_ros.py

Onkosul:
  - /mnt/managed_home/.../robot_config.json mevcut (kalibrasyon).
  - farm-ng'nin gps+imu servisleri durdurulmus (cihaz cakismasi olmasin).
  - roslibpy host venv'ine kurulu (pip install roslibpy).
  - rosbridge_websocket container icinde port 9090'da calisiyor
    (network_mode: host sayesinde host'tan da localhost:9090 erisilir).
"""
from __future__ import annotations

import argparse
import math
import socket
import struct
import sys
import threading
import time

# Gercek filter servisinin kodu burada kurulu -> UkFilterWrapper'i oradan al
sys.path.insert(0, "/opt/farmng/services/filter")

import depthai as dai  # noqa: E402
import numpy as np  # noqa: E402
import roslibpy  # noqa: E402
import serial  # noqa: E402
from farm_ng.amiga.amiga_pb2 import AmigaRobotConfig  # noqa: E402
from farm_ng.canbus import canbus_pb2  # noqa: E402
from farm_ng.core import event_pb2  # noqa: E402
from farm_ng.core.events_file_reader import proto_from_json_file  # noqa: E402
from farm_ng.core.stamp import StampSemantics, timestamp_from_monotonic  # noqa: E402
from farm_ng.gps import gps_pb2  # noqa: E402
from farm_ng.oak import oak_pb2  # noqa: E402
from farm_ng.track.utils import compute_relative_position  # noqa: E402
from ukf_wrapper import UkFilterWrapper  # noqa: E402

ROBOT_CONFIG_PATH = "/mnt/service_config/robot_config.json"

GPS_PORT, GPS_BAUD = "/dev/ttyACM0", 38400
OAK_IP = "10.95.76.10"
CAN_ID = "can0"

ROSBRIDGE_HOST = "localhost"
ROSBRIDGE_PORT = 9090
ODOM_TOPIC = "/ukf_direct/odom"
FIX_TOPIC = "/ukf_direct/fix"

# NOT: headMot (GPS course-over-ground) heading icin KULLANILMIYOR.
# headMot, aracin burnunun yonunu degil GPS noktasinin hareket yonunu verir --
# arac geri giderken bu 180 derece ters cikar (gercek filter servisi de bu
# yuzden headMot'u hic kullanmaz). Heading SADECE gyro+wheel-odom (CAN
# angular_velocity) fuzyonundan -- yani state.heading'den -- okunmali.
# (Gyro eksen donusumu RobotFilterSe2 icinde robot_config'teki imu kalibrasyonu
# ile otomatik yapiliyor, GYRO_YAW_AXIS tahmini gibi bir seye gerek yok.)


def filter_heading_to_enu_yaw(heading_rad: float) -> float:
    """gps_filter.py'deki AYNI varsayim/donusum -- farm-ng'nin filter
    servisinin north-referansli heading'i (0=kuzey) ile ayni algoritma
    (RobotFilterSe2) burada da kullanildigi icin ayni +90 derece offset'in
    gecerli olmasi BEKLENIYOR, ama DOGRUDAN cmd_vel/odom testiyle DOGRULANMADI
    -- once test et, guvenme.
    """
    return heading_rad + math.pi / 2


def ros_now() -> dict:
    t = time.time()
    sec = int(t)
    nanosec = int((t - sec) * 1e9)
    return {"sec": sec, "nanosec": nanosec}


def yaw_to_quaternion(yaw_rad: float) -> dict:
    return {
        "x": 0.0,
        "y": 0.0,
        "z": math.sin(yaw_rad / 2.0),
        "w": math.cos(yaw_rad / 2.0),
    }


class SingleAntennaUkFilterWrapper(UkFilterWrapper):
    """GPS anchor gate'i tek-antenli (moving-baseline RTK heading'i olmayan)
    kurulumlar icin gevsetir. RobotFilterSe2'de heading-only olcum enjekte
    etmenin bir yolu YOK (sadece step_gps_pos_measurement var) -- bu yuzden
    GPS burada SADECE x,y pozisyonunu duzeltir, heading hala ayri bir
    complementary tahminle (bu script'teki heading_compl) hesaplanir.
    """

    def handle_gps(self, message: gps_pb2.GpsFrame) -> None:
        if self.gps_anchor_antenna is None:
            if (
                message.horizontal_accuracy < self.gps_accuracy_thresh
                and message.position_mode >= 3
            ):
                self.gps_anchor_antenna = message
            else:
                return None

        self.last_gps_stamp = message.stamp.stamp
        relpos = compute_relative_position(self.gps_anchor_antenna, message)
        measurement = np.array([relpos[0], relpos[1]])
        std_dev = np.array([message.horizontal_accuracy, message.horizontal_accuracy])

        if not self.is_stationary and self.last_gps_stamp is not None:
            self.step_process(self.last_gps_stamp)
            self.se2_filter.step_gps_pos_measurement(measurement, std_dev)
            self.filter_state_stamp = timestamp_from_monotonic(
                StampSemantics.SERVICE_RECEIVE, self.last_gps_stamp
            )


# ── UBX (GPS) ─────────────────────────────────────────────────────────────────
UBX_SYNC1, UBX_SYNC2 = 0xB5, 0x62
NAV_PVT = (0x01, 0x07)
NAV_RELPOSNED = (0x01, 0x3C)

# ── CAN (AmigaTpdo1) ──────────────────────────────────────────────────────────
FRAME_FORMAT = "<IB3x8s"
TPDO1_FORMAT = "<BhhBBB"
TPDO1_COB_ID = 0x180 | 0xE

_lock = threading.Lock()   # UkFilterWrapper C++ state tek seferde 1 thread'den beslensin

# En son ham GPS lat/lon/hacc (UKF'in kendi ic anchor'undan BAGIMSIZ, /ukf_direct/fix icin)
_last_raw_gps = {"lat_deg": None, "lon_deg": None, "alt_m": 0.0, "hacc_m": None, "fix_type": 0}


def _checksum(data: bytes) -> tuple[int, int]:
    ck_a = ck_b = 0
    for b in data:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return ck_a, ck_b


def _read_ubx_frame(ser: serial.Serial):
    while True:
        b = ser.read(1)
        if not b:
            return None
        if b[0] != UBX_SYNC1:
            continue
        b2 = ser.read(1)
        if not b2 or b2[0] != UBX_SYNC2:
            continue
        break
    header = ser.read(4)
    if len(header) < 4:
        return None
    cls_id, msg_id, length = header[0], header[1], struct.unpack("<H", header[2:4])[0]
    payload = ser.read(length)
    chk = ser.read(2)
    if len(payload) < length or len(chk) < 2:
        return None
    if _checksum(header + payload) != (chk[0], chk[1]):
        return None
    return cls_id, msg_id, payload


def _now_stamp() -> event_pb2.Event:
    ev = event_pb2.Event()
    ev.timestamps.append(
        timestamp_from_monotonic(StampSemantics.DRIVER_RECEIVE, time.monotonic())
    )
    return ev


# ── GPS thread ────────────────────────────────────────────────────────────────

def gps_loop(wrapper: UkFilterWrapper) -> None:
    while True:
        try:
            _gps_loop_inner(wrapper)
        except (serial.SerialException, OSError) as e:
            print(f"\n[gps] hata, 2s sonra yeniden baglaniliyor: {e}")
            time.sleep(2.0)


def _gps_loop_inner(wrapper: UkFilterWrapper) -> None:
    ser = serial.Serial(GPS_PORT, GPS_BAUD, timeout=1)
    print("[gps] baglandi")
    try:
        while True:
            frame = _read_ubx_frame(ser)
            if frame is None:
                continue
            cls_id, msg_id, payload = frame
            _handle_gps_frame(wrapper, cls_id, msg_id, payload)
    finally:
        ser.close()


def _handle_gps_frame(wrapper: UkFilterWrapper, cls_id: int, msg_id: int, payload: bytes) -> None:

        if (cls_id, msg_id) == NAV_PVT and len(payload) >= 92:
            (
                _itow, _y, _mo, _d, _h, _mi, _s, _valid, _tacc, _nano,
                fix_type, _flags, _flags2, num_sv,
                lon_e7, lat_e7, _height, _hmsl, hacc, _vacc,
                _veln, _vele, _veld, _gspeed, _headmot, _sacc, _headacc,
            ) = struct.unpack_from("<IHBBBBBBiiBBBBiiiiIIiiiiIiI", payload, 0)
            # _gspeed/_headmot (GPS course-over-ground) bilerek kullanilmiyor --
            # arac geri giderken 180 derece ters cikar, bkz. dosya basindaki not.

            lat_deg = lat_e7 / 1e7
            lon_deg = lon_e7 / 1e7
            with _lock:
                _last_raw_gps["lat_deg"] = lat_deg
                _last_raw_gps["lon_deg"] = lon_deg
                _last_raw_gps["alt_m"] = _hmsl / 1000.0
                _last_raw_gps["hacc_m"] = hacc / 1000.0
                _last_raw_gps["fix_type"] = fix_type

            msg = gps_pb2.GpsFrame(
                latitude=math.radians(lat_deg),
                longitude=math.radians(lon_deg),
                horizontal_accuracy=hacc / 1000.0,
                position_mode=fix_type,
                num_satellites=num_sv,
            )
            msg.stamp.CopyFrom(
                timestamp_from_monotonic(StampSemantics.DRIVER_RECEIVE, time.monotonic())
            )
            with _lock:
                wrapper.handle_gps(msg)

        elif (cls_id, msg_id) == NAV_RELPOSNED and len(payload) >= 64:
            rel_n_cm = struct.unpack_from("<i", payload, 8)[0]
            rel_e_cm = struct.unpack_from("<i", payload, 12)[0]
            rel_heading_e5 = struct.unpack_from("<i", payload, 24)[0]
            acc_n = struct.unpack_from("<I", payload, 36)[0]
            acc_e = struct.unpack_from("<I", payload, 40)[0]
            flags = struct.unpack_from("<I", payload, 60)[0]

            msg = gps_pb2.RelativePositionFrame(
                relative_pose_north=rel_n_cm / 100.0,
                relative_pose_east=rel_e_cm / 100.0,
                relative_pose_heading=math.radians(rel_heading_e5 / 1e5),
                rel_pos_valid=bool(flags & (1 << 2)),
                carr_soln=(flags >> 3) & 0x3,
                accuracy_north=acc_n / 10000.0,
                accuracy_east=acc_e / 10000.0,
            )
            msg.stamp.CopyFrom(
                timestamp_from_monotonic(StampSemantics.DRIVER_RECEIVE, time.monotonic())
            )
            with _lock:
                wrapper.handle_relposned(msg)


# ── IMU thread ────────────────────────────────────────────────────────────────

def imu_loop(wrapper: UkFilterWrapper) -> None:
    while True:
        try:
            _imu_loop_inner(wrapper)
        except Exception as e:  # noqa: BLE001 — depthai raises various RuntimeErrors
            print(f"\n[imu] hata, 2s sonra yeniden baglaniliyor: {e}")
            time.sleep(2.0)


def _find_oak_device(retries: int = 15, delay: float = 1.0):
    for _ in range(retries):
        available = dai.Device.getAllAvailableDevices()
        device_info = next((d for d in available if d.name == OAK_IP), None)
        if device_info is not None:
            return device_info
        time.sleep(delay)
    raise RuntimeError(f"OAK {OAK_IP} {retries * delay:.0f}s icinde bulunamadi")


def _imu_loop_inner(wrapper: UkFilterWrapper) -> None:
    pipeline = dai.Pipeline()
    imu = pipeline.create(dai.node.IMU)
    imu.enableIMUSensor(
        [dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], 100
    )
    imu.setBatchReportThreshold(5)
    imu.setMaxBatchReports(10)
    xout = pipeline.create(dai.node.XLinkOut)
    xout.setStreamName("imu")
    imu.out.link(xout.input)

    device_info = _find_oak_device()

    with dai.Device(pipeline, device_info) as device:
        print("[imu] baglandi")
        q = device.getOutputQueue(name="imu", maxSize=50, blocking=False)
        while True:
            data = q.get()
            msg = oak_pb2.OakImuPackets()
            for packet in data.packets:
                g, a = packet.gyroscope, packet.acceleroMeter
                p = msg.packets.add()
                p.gyro_packet.gyro.x = g.x
                p.gyro_packet.gyro.y = g.y
                p.gyro_packet.gyro.z = g.z
                p.gyro_packet.timestamp = g.timestamp.get().total_seconds()
                p.accelero_packet.accelero.x = a.x
                p.accelero_packet.accelero.y = a.y
                p.accelero_packet.accelero.z = a.z
                p.accelero_packet.timestamp = a.timestamp.get().total_seconds()
            with _lock:
                wrapper.handle_imu(msg)


# ── CAN thread ────────────────────────────────────────────────────────────────

def can_loop(wrapper: UkFilterWrapper) -> None:
    while True:
        try:
            _can_loop_inner(wrapper)
        except OSError as e:
            print(f"\n[can] hata, 2s sonra yeniden baglaniliyor: {e}")
            time.sleep(2.0)


def _can_loop_inner(wrapper: UkFilterWrapper) -> None:
    sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.bind((CAN_ID,))
    print("[can] baglandi")
    while True:
        frame = sock.recv(16)
        cob_id, _length, data = struct.unpack(FRAME_FORMAT, frame)
        cob_id &= socket.CAN_EFF_MASK
        if cob_id != TPDO1_COB_ID:
            continue
        _state, speed_mm_s, ang_rate_mrad_s, _pto, _hbridge, _soc = struct.unpack(
            TPDO1_FORMAT, data[:8]
        )
        twist = canbus_pb2.Twist2d(
            linear_velocity_x=speed_mm_s / 1000.0,
            angular_velocity=ang_rate_mrad_s / 1000.0,
        )
        event = _now_stamp()
        with _lock:
            wrapper.handle_twist(twist, event)


# ── ROS2 publish (roslibpy -> rosbridge) ──────────────────────────────────────

def ros_publish_loop(wrapper: UkFilterWrapper, rate_hz: float = 10.0) -> None:
    ros = roslibpy.Ros(host=ROSBRIDGE_HOST, port=ROSBRIDGE_PORT)
    ros.run()
    print(f"[ros] rosbridge'e baglandi ({ROSBRIDGE_HOST}:{ROSBRIDGE_PORT})")

    odom_pub = roslibpy.Topic(ros, ODOM_TOPIC, "nav_msgs/Odometry")
    fix_pub = roslibpy.Topic(ros, FIX_TOPIC, "sensor_msgs/NavSatFix")
    odom_pub.advertise()
    fix_pub.advertise()

    dt = 1.0 / rate_hz
    try:
        while ros.is_connected:
            with _lock:
                state = wrapper.get_state()
                raw_gps = dict(_last_raw_gps)

            pose = state.pose.a_from_b
            yaw_enu = filter_heading_to_enu_yaw(state.heading)
            now = ros_now()

            odom_pub.publish(roslibpy.Message({
                "header": {"stamp": now, "frame_id": "map"},
                "child_frame_id": "base_link",
                "pose": {
                    "pose": {
                        "position": {
                            "x": float(pose.translation.x),
                            "y": float(pose.translation.y),
                            "z": 0.0,
                        },
                        "orientation": yaw_to_quaternion(yaw_enu),
                    },
                    "covariance": [0.0] * 36,
                },
                "twist": {
                    "twist": {
                        "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                        "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
                    },
                    "covariance": [0.0] * 36,
                },
            }))

            if raw_gps["lat_deg"] is not None:
                fix_pub.publish(roslibpy.Message({
                    "header": {"stamp": now, "frame_id": "gps_link"},
                    "status": {
                        "status": 0 if raw_gps["fix_type"] >= 3 else -1,
                        "service": 1,
                    },
                    "latitude": raw_gps["lat_deg"],
                    "longitude": raw_gps["lon_deg"],
                    "altitude": raw_gps["alt_m"],
                    "position_covariance": [0.0] * 9,
                    "position_covariance_type": 0,
                }))

            print(
                f"\r[ros] x={pose.translation.x:+7.3f} y={pose.translation.y:+7.3f} "
                f"yaw_enu={math.degrees(yaw_enu):6.1f}°  "
                f"lat={raw_gps['lat_deg']}  lon={raw_gps['lon_deg']}      ",
                end="", flush=True,
            )
            time.sleep(dt)
    finally:
        odom_pub.unadvertise()
        fix_pub.unadvertise()
        ros.terminate()


# ── main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description="UKF direct -> roslibpy -> rosbridge")
    parser.add_argument("--rate", type=float, default=10.0, help="ROS publish Hz")
    args = parser.parse_args()

    robot_config: AmigaRobotConfig = proto_from_json_file(
        ROBOT_CONFIG_PATH, AmigaRobotConfig()
    )
    # NTRIP/RTK duzeltmesi yok -> gercek hAcc ~0.5m. 11mm esigi hicbir zaman
    # gecilemezdi, bu yuzden normal (RTK'siz) GPS icin gercekci bir esik kullan.
    wrapper = SingleAntennaUkFilterWrapper(robot_config, gps_accuracy_thresh=1.5)

    threading.Thread(target=gps_loop, args=(wrapper,), daemon=True).start()
    threading.Thread(target=imu_loop, args=(wrapper,), daemon=True).start()
    threading.Thread(target=can_loop, args=(wrapper,), daemon=True).start()

    print(
        "UKF dogrudan-donanim modunda calisiyor, roslibpy ile "
        f"{ODOM_TOPIC} ve {FIX_TOPIC}'e basiliyor (Ctrl+C ile cik)\n"
    )
    try:
        ros_publish_loop(wrapper, rate_hz=args.rate)
    except KeyboardInterrupt:
        print("\nDurduruldu.")


if __name__ == "__main__":
    main()
