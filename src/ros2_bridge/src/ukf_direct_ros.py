#!/usr/bin/env python3
"""Gercek farm_ng UKF'sini (UkFilterWrapper + RobotFilterSe2) tek bir process'te
calistirir, GPS/IMU/CAN verisini gRPC/protobuf-over-network yerine DOGRUDAN
donanimdan okur, sonucu roslibpy ile mevcut rosbridge_server (container
icinde, port 9090) uzerinden ROS2 topic'lerine publish eder, VE /cmd_vel'i
dinleyip dogrudan CAN'a (dashboard'a) hiz komutu basar.

UYARI -- BU SCRIPT ROBOTU GERCEKTEN HAREKET ETTIREBILIR:
  /ukf_direct/auto_mode (std_msgs/Bool, data=true) topic'ine mesaj
  gonderildiginde robot AUTO moda gecer ve /cmd_vel'den gelen komutlari
  CAN'a basmaya baslar. data=false gonderilince (ya da /cmd_vel
  CMD_VEL_TIMEOUT_S suresinden uzun sure gelmezse -- dead-man's switch)
  sifir hiz gonderilip AUTO mod birakilir. Once acik/guvenli bir alanda,
  dusuk hizla test et.

Bu, gps_filter.py + ros2_to_twist.py + farm-ng'nin kendi gps/imu/canbus/
filter servislerinin YERINE GECMEZ -- onlarla AYNI ANDA, AYNI donanimi
(GPS seri port, OAK IMU, can0) acmaya calisirsa cakisma olur:
  - GPS/IMU: farm-ng'nin gps+imu servisleri durdurulmus olmali.
  - CAN: hem okuma (wheel odom) hem yazma (RPDO1) yapiyoruz -- farm-ng'nin
    canbus servisi ÇALIŞIRKEN AUTO mode'a gecmeye calisirsak dashboard'a
    CAKISAN iki kaynaktan komut gidebilir (GUVENLIK RISKI). canbus
    servisi de durdurulmus olmali, /cmd_vel testi yapmadan once.

Sonuc, /rtk/odom ve /gps/fix'e BASIYOR -- gps_filter.py'nin normalde
bastigi topic'lerle AYNI isim. gps_filter.py o anda farm-ng servislerine
baglanamadigi icin (servisler durdurulmus) fiilen veri basmiyor olmali,
ama ikisini AYNI ANDA calistirmamaya dikkat et (cift publisher).

ROBOT (Jetson) UZERINDE, HOST'ta /farm_ng_image/venv ile calistirilmali
(container icinde DEGIL -- depthai/ukf_wrapper bu venv'e ozel):
    /farm_ng_image/venv/bin/python3 ukf_direct_ros.py

Onkosul:
  - /mnt/managed_home/.../robot_config.json mevcut (kalibrasyon).
  - farm-ng'nin gps+imu+canbus servisleri durdurulmus.
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

# gps_filter.py'deki AYNI sabit anchor -- /rtk/odom'un pozisyonu BUNA göre
# hesaplanir, UKF'in kendi (NWU, pose.translation) cikisina göre DEGIL.
# NEDEN: farm_ng.track.utils.compute_relative_position NWU (north,-east,-down)
# dönüyor; se2_filter.step_gps_pos_measurement de bu [north,west] ölçümle
# besleniyor, yani UKF'in world_from_robot.translation.x/y'si ENU (x=doğu,
# y=kuzey) DEGIL. gps_filter.py bunu zaten biliyordu ve pose.translation'ı
# hiç kullanmiyordu, heading'i filtreden alip pozisyonu ham GPS lat/lon'dan
# kendi ENU projeksiyonuyla hesapliyordu -- ayni korumayi burada da
# uyguluyoruz.
DEFAULT_ANCHOR_LAT_DEG = 39.796011
DEFAULT_ANCHOR_LON_DEG = 32.531534
EARTH_RADIUS_M = 6378137.0


def latlon_to_local_xy(lat_deg: float, lon_deg: float, anchor_lat_deg: float, anchor_lon_deg: float) -> tuple[float, float]:
    """WGS84 lat/lon'u sabit anchor'a göre yerel ENU (x=doğu, y=kuzey) metreye çevirir."""
    anchor_lat_rad = math.radians(anchor_lat_deg)
    dlat = math.radians(lat_deg - anchor_lat_deg)
    dlon = math.radians(lon_deg - anchor_lon_deg)
    x_east = dlon * math.cos(anchor_lat_rad) * EARTH_RADIUS_M
    y_north = dlat * EARTH_RADIUS_M
    return x_east, y_north


GPS_PORT, GPS_BAUD = "/dev/ttyACM0", 38400
OAK_IP = "10.95.76.11"  # NOT: ag uzerinde 10.95.76.10 ve .11 de gorulebilir --
                         # dai.Device.getAllAvailableDevices() ile dogrula, gerekirse degistir.
CAN_ID = "can0"

ROSBRIDGE_HOST = "localhost"
ROSBRIDGE_PORT = 9090
ODOM_TOPIC = "/rtk/odom"
FIX_TOPIC = "/gps/fix"
ANCHOR_TOPIC = "/anchor"
CMD_VEL_TOPIC = "/cmd_vel"
AUTO_MODE_TOPIC = "/ukf_direct/auto_mode"

# ── CAN gonderme (RPDO1 / heartbeat / auto-mode istegi) ───────────────────────
# wasd_drive.py / send_twist_direct.py ile AYNI protokol.
DASHBOARD_NODE_ID = 0xE
AMIGA_BRAIN_ID = 0x1F
RPDO1_COB_ID = 0x200 | DASHBOARD_NODE_ID          # 0x20E
RPDO1_FORMAT = "<BhhBBx"
STATE_AUTO_ACTIVE = 5
HEARTBEAT_COB_ID = 0x700 | AMIGA_BRAIN_ID         # 0x71F
HEARTBEAT_FORMAT = "<BI3s"
NODE_STATE_OPERATIONAL = 0x05
REQREP_COB_ID_REQ = 0x600 | DASHBOARD_NODE_ID     # 0x60E
REQREP_FORMAT = "<BHx4s"
OP_WRITE = 2
VAL_REQUEST_AUTO_MODE = 1001
UNIT_NA = 1

CMD_SEND_RATE_HZ = 20.0
HEARTBEAT_RATE_HZ = 2.0
CMD_VEL_TIMEOUT_S = 0.5   # dead-man's switch: bu sureden uzun /cmd_vel gelmezse sifir hiz
MAX_SPEED = 0.5           # m/s hard cap (guvenlik)
MAX_ANGULAR = 0.5         # rad/s hard cap

# UKF GPS olcum gurultusu ayari -- bkz. SingleAntennaUkFilterWrapper.handle_gps
GPS_STD_DEV_SCALE = 0.15  # hAcc'in carpani: dusuk = filtre GPS'e daha cok guvenir (daha hizli yakinsama, biraz daha titrek)
RTK_MIN_STD_DEV_M = 0.01  # sayisal guvenlik alt siniri (RTK ile hAcc ~0'a yaklasinca)

# NOT: headMot (GPS course-over-ground) heading icin KULLANILMIYOR.
# headMot, aracin burnunun yonunu degil GPS noktasinin hareket yonunu verir --
# arac geri giderken bu 180 derece ters cikar (gercek filter servisi de bu
# yuzden headMot'u hic kullanmaz). Heading SADECE gyro+wheel-odom (CAN
# angular_velocity) fuzyonundan -- yani state.heading'den -- okunmali.


def filter_heading_to_enu_yaw(heading_rad: float) -> float:
    """gps_filter.py'deki AYNI varsayim/donusum -- farm-ng'nin filter
    servisinin north-referansli heading'i (0=kuzey) ile ayni algoritma
    (RobotFilterSe2) burada da kullanildigi icin ayni +90 derece offset'in
    gecerli olmasi BEKLENIYOR, ama BU SCRIPT icin DOGRUDAN DOGRULANMADI --
    once cmd_vel/odom karsilastirmasiyla test et, guvenme.
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
        # NOT: hAcc'i oldugu gibi degil, GPS_STD_DEV_SCALE ile carpip kullaniyoruz --
        # filtreye GPS'e raporlanandan biraz daha fazla guven (daha hizli
        # drift-duzeltme), RTK_MIN_STD_DEV_M ise sayisal guvenlik icin alt sinir
        # (hAcc RTK ile ~1-2cm'e dustugunde sifira yakin std_dev Kalman gain'i
        # bozabilir). RTK geldiginde hAcc kucalecegi icin bu carpan otomatik
        # olarak daha siki takip verir, yeniden ayara gerek kalmaz.
        hacc = max(message.horizontal_accuracy * GPS_STD_DEV_SCALE, RTK_MIN_STD_DEV_M)
        std_dev = np.array([hacc, hacc])

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

# ── CAN okuma (AmigaTpdo1 -- tekerlek odometrisi) ─────────────────────────────
FRAME_FORMAT = "<IB3x8s"
TPDO1_FORMAT = "<BhhBBB"
TPDO1_COB_ID = 0x180 | 0xE

_lock = threading.Lock()   # UkFilterWrapper C++ state tek seferde 1 thread'den beslensin

# En son ham GPS lat/lon/hacc (UKF'in kendi ic anchor'undan BAGIMSIZ, /gps/fix icin)
_last_raw_gps = {"lat_deg": None, "lon_deg": None, "alt_m": 0.0, "hacc_m": None, "fix_type": 0}

# En son CAN'dan okunan tekerlek-odometri hizi (/rtk/odom'un twist alani icin)
_last_can_twist = {"linear_x": 0.0, "angular_z": 0.0, "stamp": 0.0}

# /cmd_vel'den gelen son komut + ne zaman geldigi (dead-man's switch icin) ve auto-mode durumu
_cmd_state = {"linear_x": 0.0, "angular_z": 0.0, "last_rx": 0.0, "auto_mode": False}
_cmd_lock = threading.Lock()


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


# ── CAN okuma thread'i (tekerlek odometrisi) ──────────────────────────────────

def can_read_loop(wrapper: UkFilterWrapper) -> None:
    while True:
        try:
            _can_read_loop_inner(wrapper)
        except OSError as e:
            print(f"\n[can-rx] hata, 2s sonra yeniden baglaniliyor: {e}")
            time.sleep(2.0)


def _can_read_loop_inner(wrapper: UkFilterWrapper) -> None:
    sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.bind((CAN_ID,))
    print("[can-rx] baglandi")
    while True:
        frame = sock.recv(16)
        cob_id, _length, data = struct.unpack(FRAME_FORMAT, frame)
        cob_id &= socket.CAN_EFF_MASK
        if cob_id != TPDO1_COB_ID:
            continue
        _state, speed_mm_s, ang_rate_mrad_s, _pto, _hbridge, _soc = struct.unpack(
            TPDO1_FORMAT, data[:8]
        )
        linear_x = speed_mm_s / 1000.0
        angular_z = ang_rate_mrad_s / 1000.0
        with _lock:
            _last_can_twist["linear_x"] = linear_x
            _last_can_twist["angular_z"] = angular_z
            _last_can_twist["stamp"] = time.monotonic()

        twist = canbus_pb2.Twist2d(
            linear_velocity_x=linear_x,
            angular_velocity=angular_z,
        )
        event = _now_stamp()
        with _lock:
            wrapper.handle_twist(twist, event)


# ── CAN yazma thread'i (RPDO1 + heartbeat + auto-mode istegi) ─────────────────
# wasd_drive.py / send_twist_direct.py ile AYNI protokol, ama tetikleyici
# /cmd_vel (ROS) ve /ukf_direct/auto_mode (ROS) -- klavye degil.

def _can_send(sock: socket.socket, cob_id: int, data: bytes) -> None:
    frame = struct.pack(FRAME_FORMAT, cob_id, len(data), data)
    sock.send(frame)


def _rpdo1_bytes(speed: float, ang_rate: float) -> bytes:
    speed = max(-MAX_SPEED, min(MAX_SPEED, speed))
    ang_rate = max(-MAX_ANGULAR, min(MAX_ANGULAR, ang_rate))
    return struct.pack(
        RPDO1_FORMAT, STATE_AUTO_ACTIVE,
        int(speed * 1000.0), int(ang_rate * 1000.0),
        0x0, 0x0,
    )


def _heartbeat_bytes() -> bytes:
    ticks_ms = int(time.monotonic() * 1000) & 0xFFFFFFFF
    return struct.pack(HEARTBEAT_FORMAT, NODE_STATE_OPERATIONAL, ticks_ms, b"\x00\x00\x00")


def _request_auto_mode_bytes(enable: bool) -> bytes:
    payload = struct.pack("<B3x", 1 if enable else 0)
    val_and_units = VAL_REQUEST_AUTO_MODE | (UNIT_NA << 11)
    return struct.pack(REQREP_FORMAT, OP_WRITE, val_and_units, payload)


def can_write_loop() -> None:
    """/cmd_vel + /ukf_direct/auto_mode'dan beslenen, surekli 20Hz RPDO1 +
    2Hz heartbeat gonderen thread. Dead-man's switch: /cmd_vel
    CMD_VEL_TIMEOUT_S'den uzun sure gelmezse hiz sifirlanir (AUTO mod
    birakilmaz, sadece hiz sifirlanir -- auto_mode acikken robot "auto
    active + sifir hiz" durumunda kalir, kapatmak icin /ukf_direct/auto_mode
    data=false gerekir).
    """
    sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.bind((CAN_ID,))
    print("[can-tx] baglandi (RPDO1/heartbeat hazir, auto_mode=false bekleniyor)")

    cmd_dt = 1.0 / CMD_SEND_RATE_HZ
    hb_dt = 1.0 / HEARTBEAT_RATE_HZ
    last_hb = 0.0
    last_auto_mode_sent = None

    while True:
        now = time.monotonic()
        with _cmd_lock:
            auto_mode = _cmd_state["auto_mode"]
            linear_x = _cmd_state["linear_x"]
            angular_z = _cmd_state["angular_z"]
            last_rx = _cmd_state["last_rx"]

        if auto_mode != last_auto_mode_sent:
            _can_send(sock, REQREP_COB_ID_REQ, _request_auto_mode_bytes(auto_mode))
            print(f"\n[can-tx] AUTO_MODE istegi gonderildi: {auto_mode}")
            last_auto_mode_sent = auto_mode

        if auto_mode:
            stale = (now - last_rx) > CMD_VEL_TIMEOUT_S
            if stale:
                linear_x, angular_z = 0.0, 0.0
            _can_send(sock, RPDO1_COB_ID, _rpdo1_bytes(linear_x, angular_z))
            if now - last_hb >= hb_dt:
                _can_send(sock, HEARTBEAT_COB_ID, _heartbeat_bytes())
                last_hb = now

        time.sleep(cmd_dt)


def on_cmd_vel(message) -> None:
    linear = message.get("linear", {})
    angular = message.get("angular", {})
    with _cmd_lock:
        _cmd_state["linear_x"] = float(linear.get("x", 0.0))
        _cmd_state["angular_z"] = float(angular.get("z", 0.0))
        _cmd_state["last_rx"] = time.monotonic()


def on_auto_mode(message) -> None:
    enable = bool(message.get("data", False))
    with _cmd_lock:
        _cmd_state["auto_mode"] = enable
        if not enable:
            _cmd_state["linear_x"] = 0.0
            _cmd_state["angular_z"] = 0.0


# ── ROS2 publish + subscribe (roslibpy <-> rosbridge) ─────────────────────────

def ros_loop(wrapper: UkFilterWrapper, rate_hz: float = 10.0) -> None:
    ros = roslibpy.Ros(host=ROSBRIDGE_HOST, port=ROSBRIDGE_PORT)
    ros.run()
    print(f"[ros] rosbridge'e baglandi ({ROSBRIDGE_HOST}:{ROSBRIDGE_PORT})")

    odom_pub = roslibpy.Topic(ros, ODOM_TOPIC, "nav_msgs/Odometry")
    fix_pub = roslibpy.Topic(ros, FIX_TOPIC, "sensor_msgs/NavSatFix")
    anchor_pub = roslibpy.Topic(ros, ANCHOR_TOPIC, "sensor_msgs/NavSatFix")
    odom_pub.advertise()
    fix_pub.advertise()
    anchor_pub.advertise()

    cmd_vel_sub = roslibpy.Topic(ros, CMD_VEL_TOPIC, "geometry_msgs/Twist")
    auto_mode_sub = roslibpy.Topic(ros, AUTO_MODE_TOPIC, "std_msgs/Bool")
    cmd_vel_sub.subscribe(on_cmd_vel)
    auto_mode_sub.subscribe(on_auto_mode)
    print(f"[ros] {CMD_VEL_TOPIC} ve {AUTO_MODE_TOPIC} dinleniyor")

    dt = 1.0 / rate_hz
    try:
        while ros.is_connected:
            with _lock:
                state = wrapper.get_state()
                raw_gps = dict(_last_raw_gps)
                can_twist = dict(_last_can_twist)
                anchor = wrapper.gps_anchor_antenna

            yaw_enu = filter_heading_to_enu_yaw(state.heading)
            now = ros_now()

            if raw_gps["lat_deg"] is not None:
                # NOT: UKF'in pose.translation'i NWU, dogrudan ENU x/y olarak
                # KULLANILMIYOR (bkz. DEFAULT_ANCHOR_LAT_DEG yorumu). Pozisyon
                # ham GPS lat/lon'dan, gps_filter.py ile AYNI sabit anchor'a
                # göre hesaplaniyor; heading hala UKF'ten (gyro+wheel-odom
                # füzyonu, GPS'ten cok daha az gürültülü).
                x_east, y_north = latlon_to_local_xy(
                    raw_gps["lat_deg"], raw_gps["lon_deg"],
                    DEFAULT_ANCHOR_LAT_DEG, DEFAULT_ANCHOR_LON_DEG,
                )
                odom_pub.publish(roslibpy.Message({
                    "header": {"stamp": now, "frame_id": "map"},
                    "child_frame_id": "base_link",
                    "pose": {
                        "pose": {
                            "position": {"x": x_east, "y": y_north, "z": 0.0},
                            "orientation": yaw_to_quaternion(yaw_enu),
                        },
                        "covariance": [0.0] * 36,
                    },
                    "twist": {
                        "twist": {
                            "linear": {"x": can_twist["linear_x"], "y": 0.0, "z": 0.0},
                            "angular": {"x": 0.0, "y": 0.0, "z": can_twist["angular_z"]},
                        },
                        "covariance": [0.0] * 36,
                    },
                }))
            else:
                x_east, y_north = 0.0, 0.0

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

            if anchor is not None:
                anchor_pub.publish(roslibpy.Message({
                    "header": {"stamp": now, "frame_id": "gps_link"},
                    "status": {"status": 0, "service": 1},
                    "latitude": math.degrees(anchor.latitude),
                    "longitude": math.degrees(anchor.longitude),
                    "altitude": anchor.altitude,
                    "position_covariance": [0.0] * 9,
                    "position_covariance_type": 0,
                }))

            with _cmd_lock:
                auto_mode = _cmd_state["auto_mode"]
            print(
                f"\r[ros] x={x_east:+7.3f} y={y_north:+7.3f} "
                f"yaw_enu={math.degrees(yaw_enu):6.1f}°  auto_mode={auto_mode}  "
                f"lat={raw_gps['lat_deg']}  lon={raw_gps['lon_deg']}      ",
                end="", flush=True,
            )
            time.sleep(dt)
    finally:
        odom_pub.unadvertise()
        fix_pub.unadvertise()
        anchor_pub.unadvertise()
        cmd_vel_sub.unsubscribe()
        auto_mode_sub.unsubscribe()
        ros.terminate()


# ── main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description="UKF direct -> roslibpy -> rosbridge + cmd_vel -> CAN")
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
    threading.Thread(target=can_read_loop, args=(wrapper,), daemon=True).start()
    threading.Thread(target=can_write_loop, daemon=True).start()

    print(
        "UKF dogrudan-donanim modunda calisiyor. roslibpy ile "
        f"{ODOM_TOPIC}, {FIX_TOPIC}, {ANCHOR_TOPIC}'e basiliyor; "
        f"{CMD_VEL_TOPIC} + {AUTO_MODE_TOPIC} dinleniyor.\n"
        "UYARI: auto_mode=true gonderilirse robot /cmd_vel'e gore HAREKET EDER.\n"
        "(Ctrl+C ile cik)\n"
    )
    try:
        ros_loop(wrapper, rate_hz=args.rate)
    except KeyboardInterrupt:
        print("\nDurduruldu.")


if __name__ == "__main__":
    main()
