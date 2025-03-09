import json
import requests
from networktables import NetworkTables
from wpimath.geometry import Pose2d, Pose3d, Rotation2d, Rotation3d, Translation2d, Translation3d
from wpimath.util import Units
from typing import List, Dict, Optional

class LimelightHelpers:
    double_array_entries: Dict[str, NetworkTables.DoubleArrayEntry] = {}

    @staticmethod
    def sanitize_name(name: str) -> str:
        return "limelight" if not name else name

    @staticmethod
    def to_pose3d(data: List[float]) -> Pose3d:
        if len(data) < 6:
            return Pose3d()
        return Pose3d(
            Translation3d(data[0], data[1], data[2]),
            Rotation3d(Units.degreesToRadians(data[3]), Units.degreesToRadians(data[4]), Units.degreesToRadians(data[5]))
        )

    @staticmethod
    def to_pose2d(data: List[float]) -> Pose2d:
        if len(data) < 6:
            return Pose2d()
        return Pose2d(
            Translation2d(data[0], data[1]),
            Rotation2d(Units.degreesToRadians(data[5]))
        )

    @staticmethod
    def pose3d_to_array(pose: Pose3d) -> List[float]:
        return [
            pose.translation().x(),
            pose.translation().y(),
            pose.translation().z(),
            Units.radiansToDegrees(pose.rotation().x()),
            Units.radiansToDegrees(pose.rotation().y()),
            Units.radiansToDegrees(pose.rotation().z())
        ]

    @staticmethod
    def pose2d_to_array(pose: Pose2d) -> List[float]:
        return [
            pose.translation().x(),
            pose.translation().y(),
            0,
            0,
            0,
            Units.radiansToDegrees(pose.rotation().radians())
        ]

    @staticmethod
    def extract_array_entry(data: List[float], position: int) -> float:
        return data[position] if len(data) > position else 0

    @staticmethod
    def get_limelight_nt_table(table_name: str) -> NetworkTables.NetworkTable:
        return NetworkTables.getTable(LimelightHelpers.sanitize_name(table_name))

    @staticmethod
    def get_limelight_nt_table_entry(table_name: str, entry_name: str) -> NetworkTables.NetworkTableEntry:
        return LimelightHelpers.get_limelight_nt_table(table_name).getEntry(entry_name)

    @staticmethod
    def get_limelight_double_array_entry(table_name: str, entry_name: str) -> NetworkTables.DoubleArrayEntry:
        key = f"{table_name}/{entry_name}"
        if key not in LimelightHelpers.double_array_entries:
            table = LimelightHelpers.get_limelight_nt_table(table_name)
            LimelightHelpers.double_array_entries[key] = table.getDoubleArrayTopic(entry_name).getEntry([])
        return LimelightHelpers.double_array_entries[key]

    @staticmethod
    def get_limelight_nt_double(table_name: str, entry_name: str) -> float:
        return LimelightHelpers.get_limelight_nt_table_entry(table_name, entry_name).getDouble(0.0)

    @staticmethod
    def set_limelight_nt_double(table_name: str, entry_name: str, val: float) -> None:
        LimelightHelpers.get_limelight_nt_table_entry(table_name, entry_name).setDouble(val)

    @staticmethod
    def set_limelight_nt_double_array(table_name: str, entry_name: str, val: List[float]) -> None:
        LimelightHelpers.get_limelight_nt_table_entry(table_name, entry_name).setDoubleArray(val)

    @staticmethod
    def get_limelight_nt_double_array(table_name: str, entry_name: str) -> List[float]:
        return LimelightHelpers.get_limelight_nt_table_entry(table_name, entry_name).getDoubleArray([])

    @staticmethod
    def get_limelight_nt_string(table_name: str, entry_name: str) -> str:
        return LimelightHelpers.get_limelight_nt_table_entry(table_name, entry_name).getString("")

    @staticmethod
    def get_limelight_nt_string_array(table_name: str, entry_name: str) -> List[str]:
        return LimelightHelpers.get_limelight_nt_table_entry(table_name, entry_name).getStringArray([])

    @staticmethod
    def get_limelight_url_string(table_name: str, request: str) -> Optional[str]:
        url_string = f"http://{LimelightHelpers.sanitize_name(table_name)}.local:5807/{request}"
        try:
            return url_string
        except Exception as e:
            print(f"Bad LL URL: {e}")
            return None

    @staticmethod
    def get_tv(limelight_name: str) -> bool:
        return LimelightHelpers.get_limelight_nt_double(limelight_name, "tv") == 1.0

    @staticmethod
    def get_tx(limelight_name: str) -> float:
        return LimelightHelpers.get_limelight_nt_double(limelight_name, "tx")

    @staticmethod
    def get_ty(limelight_name: str) -> float:
        return LimelightHelpers.get_limelight_nt_double(limelight_name, "ty")

    @staticmethod
    def get_txnc(limelight_name: str) -> float:
        return LimelightHelpers.get_limelight_nt_double(limelight_name, "txnc")

    @staticmethod
    def get_tync(limelight_name: str) -> float:
        return LimelightHelpers.get_limelight_nt_double(limelight_name, "tync")

    @staticmethod
    def get_ta(limelight_name: str) -> float:
        return LimelightHelpers.get_limelight_nt_double(limelight_name, "ta")

    @staticmethod
    def get_t2d_array(limelight_name: str) -> List[float]:
        return LimelightHelpers.get_limelight_nt_double_array(limelight_name, "t2d")

    @staticmethod
    def get_target_count(limelight_name: str) -> int:
        t2d = LimelightHelpers.get_t2d_array(limelight_name)
        return int(t2d[1]) if len(t2d) == 17 else 0

    @staticmethod
    def get_classifier_class_index(limelight_name: str) -> int:
        t2d = LimelightHelpers.get_t2d_array(limelight_name)
        return int(t2d[10]) if len(t2d) == 17 else 0

    @staticmethod
    def get_detector_class_index(limelight_name: str) -> int:
        t2d = LimelightHelpers.get_t2d_array(limelight_name)
        return int(t2d[11]) if len(t2d) == 17 else 0

    @staticmethod
    def get_classifier_class(limelight_name: str) -> str:
        return LimelightHelpers.get_limelight_nt_string(limelight_name, "tcclass")

    @staticmethod
    def get_detector_class(limelight_name: str) -> str:
        return LimelightHelpers.get_limelight_nt_string(limelight_name, "tdclass")

    @staticmethod
    def get_latency_pipeline(limelight_name: str) -> float:
        return LimelightHelpers.get_limelight_nt_double(limelight_name, "tl")

    @staticmethod
    def get_latency_capture(limelight_name: str) -> float:
        return LimelightHelpers.get_limelight_nt_double(limelight_name, "cl")

    @staticmethod
    def get_current_pipeline_index(limelight_name: str) -> float:
        return LimelightHelpers.get_limelight_nt_double(limelight_name, "getpipe")

    @staticmethod
    def get_current_pipeline_type(limelight_name: str) -> str:
        return LimelightHelpers.get_limelight_nt_string(limelight_name, "getpipetype")

    @staticmethod
    def get_json_dump(limelight_name: str) -> str:
        return LimelightHelpers.get_limelight_nt_string(limelight_name, "json")

    @staticmethod
    def get_bot_pose(limelight_name: str) -> List[float]:
        return LimelightHelpers.get_limelight_nt_double_array(limelight_name, "botpose")

    @staticmethod
    def get_bot_pose_wpi_red(limelight_name: str) -> List[float]:
        return LimelightHelpers.get_limelight_nt_double_array(limelight_name, "botpose_wpired")

    @staticmethod
    def get_bot_pose_wpi_blue(limelight_name: str) -> List[float]:
        return LimelightHelpers.get_limelight_nt_double_array(limelight_name, "botpose_wpiblue")

    @staticmethod
    def get_bot_pose_target_space(limelight_name: str) -> List[float]:
        return LimelightHelpers.get_limelight_nt_double_array(limelight_name, "botpose_targetspace")

    @staticmethod
    def get_camera_pose_target_space(limelight_name: str) -> List[float]:
        return LimelightHelpers.get_limelight_nt_double_array(limelight_name, "camerapose_targetspace")

    @staticmethod
    def get_target_pose_camera_space(limelight_name: str) -> List[float]:
        return LimelightHelpers.get_limelight_nt_double_array(limelight_name, "targetpose_cameraspace")

    @staticmethod
    def get_target_pose_robot_space(limelight_name: str) -> List[float]:
        return LimelightHelpers.get_limelight_nt_double_array(limelight_name, "targetpose_robotspace")

    @staticmethod
    def get_target_color(limelight_name: str) -> List[float]:
        return LimelightHelpers.get_limelight_nt_double_array(limelight_name, "tc")

    @staticmethod
    def get_fiducial_id(limelight_name: str) -> float:
        return LimelightHelpers.get_limelight_nt_double(limelight_name, "tid")

    @staticmethod
    def get_neural_class_id(limelight_name: str) -> str:
        return LimelightHelpers.get_limelight_nt_string(limelight_name, "tclass")

    @staticmethod
    def get_raw_barcode_data(limelight_name: str) -> List[str]:
        return LimelightHelpers.get_limelight_nt_string_array(limelight_name, "rawbarcodes")

    @staticmethod
    def get_bot_pose3d(limelight_name: str) -> Pose3d:
        pose_array = LimelightHelpers.get_bot_pose(limelight_name)
        return LimelightHelpers.to_pose3d(pose_array)

    @staticmethod
    def get_bot_pose3d_wpi_red(limelight_name: str) -> Pose3d:
        pose_array = LimelightHelpers.get_bot_pose_wpi_red(limelight_name)
        return LimelightHelpers.to_pose3d(pose_array)

    @staticmethod
    def get_bot_pose3d_wpi_blue(limelight_name: str) -> Pose3d:
        pose_array = LimelightHelpers.get_bot_pose_wpi_blue(limelight_name)
        return LimelightHelpers.to_pose3d(pose_array)

    @staticmethod
    def get_bot_pose3d_target_space(limelight_name: str) -> Pose3d:
        pose_array = LimelightHelpers.get_bot_pose_target_space(limelight_name)
        return LimelightHelpers.to_pose3d(pose_array)

    @staticmethod
    def get_camera_pose3d_target_space(limelight_name: str) -> Pose3d:
        pose_array = LimelightHelpers.get_camera_pose_target_space(limelight_name)
        return LimelightHelpers.to_pose3d(pose_array)

    @staticmethod
    def get_target_pose3d_camera_space(limelight_name: str) -> Pose3d:
        pose_array = LimelightHelpers.get_target_pose_camera_space(limelight_name)
        return LimelightHelpers.to_pose3d(pose_array)

    @staticmethod
    def get_target_pose3d_robot_space(limelight_name: str) -> Pose3d:
        pose_array = LimelightHelpers.get_target_pose_robot_space(limelight_name)
        return LimelightHelpers.to_pose3d(pose_array)

    @staticmethod
    def get_camera_pose3d_robot_space(limelight_name: str) -> Pose3d:
        pose_array = LimelightHelpers.get_limelight_nt_double_array(limelight_name, "camerapose_robotspace")
        return LimelightHelpers.to_pose3d(pose_array)

    @staticmethod
    def get_bot_pose2d_wpi_blue(limelight_name: str) -> Pose2d:
        result = LimelightHelpers.get_bot_pose_wpi_blue(limelight_name)
        return LimelightHelpers.to_pose2d(result)

    @staticmethod
    def get_bot_pose_estimate_wpi_blue(limelight_name: str) -> 'PoseEstimate':
        return LimelightHelpers.get_bot_pose_estimate(limelight_name, "botpose_wpiblue", False)

    @staticmethod
    def get_bot_pose_estimate_wpi_blue_megatag2(limelight_name: str) -> 'PoseEstimate':
        return LimelightHelpers.get_bot_pose_estimate(limelight_name, "botpose_orb_wpiblue", True)

    @staticmethod
    def get_bot_pose2d_wpi_red(limelight_name: str) -> Pose2d:
        result = LimelightHelpers.get_bot_pose_wpi_red(limelight_name)
        return LimelightHelpers.to_pose2d(result)

    @staticmethod
    def get_bot_pose_estimate_wpi_red(limelight_name: str) -> 'PoseEstimate':
        return LimelightHelpers.get_bot_pose_estimate(limelight_name, "botpose_wpired", False)

    @staticmethod
    def get_bot_pose_estimate_wpi_red_megatag2(limelight_name: str) -> 'PoseEstimate':
        return LimelightHelpers.get_bot_pose_estimate(limelight_name, "botpose_orb_wpired", True)

    @staticmethod
    def get_bot_pose2d(limelight_name: str) -> Pose2d:
        result = LimelightHelpers.get_bot_pose(limelight_name)
        return LimelightHelpers.to_pose2d(result)

    @staticmethod
    def get_imu_data(limelight_name: str) -> 'IMUData':
        imu_data = LimelightHelpers.get_limelight_nt_double_array(limelight_name, "imu")
        return IMUData(imu_data) if imu_data and len(imu_data) >= 10 else IMUData()

    @staticmethod
    def set_pipeline_index(limelight_name: str, pipeline_index: int) -> None:
        LimelightHelpers.set_limelight_nt_double(limelight_name, "pipeline", pipeline_index)

    @staticmethod
    def set_priority_tag_id(limelight_name: str, tag_id: int) -> None:
        LimelightHelpers.set_limelight_nt_double(limelight_name, "priorityid", tag_id)

    @staticmethod
    def set_led_mode_pipeline_control(limelight_name: str) -> None:
        LimelightHelpers.set_limelight_nt_double(limelight_name, "ledMode", 0)

    @staticmethod
    def set_led_mode_force_off(limelight_name: str) -> None:
        LimelightHelpers.set_limelight_nt_double(limelight_name, "ledMode", 1)

    @staticmethod
    def set_led_mode_force_blink(limelight_name: str) -> None:
        LimelightHelpers.set_limelight_nt_double(limelight_name, "ledMode", 2)

    @staticmethod
    def set_led_mode_force_on(limelight_name: str) -> None:
        LimelightHelpers.set_limelight_nt_double(limelight_name, "ledMode", 3)

    @staticmethod
    def set_stream_mode_standard(limelight_name: str) -> None:
        LimelightHelpers.set_limelight_nt_double(limelight_name, "stream", 0)

    @staticmethod
    def set_stream_mode_pip_main(limelight_name: str) -> None:
        LimelightHelpers.set_limelight_nt_double(limelight_name, "stream", 1)

    @staticmethod
    def set_stream_mode_pip_secondary(limelight_name: str) -> None:
        LimelightHelpers.set_limelight_nt_double(limelight_name, "stream", 2)

    @staticmethod
    def set_crop_window(limelight_name: str, crop_x_min: float, crop_x_max: float, crop_y_min: float, crop_y_max: float) -> None:
        entries = [crop_x_min, crop_x_max, crop_y_min, crop_y_max]
        LimelightHelpers.set_limelight_nt_double_array(limelight_name, "crop", entries)

    @staticmethod
    def set_fiducial_3d_offset(limelight_name: str, offset_x: float, offset_y: float, offset_z: float) -> None:
        entries = [offset_x, offset_y, offset_z]
        LimelightHelpers.set_limelight_nt_double_array(limelight_name, "fiducial_offset_set", entries)

    @staticmethod
    def set_robot_orientation(limelight_name: str, yaw: float, yaw_rate: float, pitch: float, pitch_rate: float, roll: float, roll_rate: float) -> None:
        LimelightHelpers.set_robot_orientation_internal(limelight_name, yaw, yaw_rate, pitch, pitch_rate, roll, roll_rate, True)

    @staticmethod
    def set_robot_orientation_no_flush(limelight_name: str, yaw: float, yaw_rate: float, pitch: float, pitch_rate: float, roll: float, roll_rate: float) -> None:
        LimelightHelpers.set_robot_orientation_internal(limelight_name, yaw, yaw_rate, pitch, pitch_rate, roll, roll_rate, False)

    @staticmethod
    def set_robot_orientation_internal(limelight_name: str, yaw: float, yaw_rate: float, pitch: float, pitch_rate: float, roll: float, roll_rate: float, flush: bool) -> None:
        entries = [yaw, yaw_rate, pitch, pitch_rate, roll, roll_rate]
        LimelightHelpers.set_limelight_nt_double_array(limelight_name, "robot_orientation_set", entries)
        if flush:
            NetworkTables.flush()

    @staticmethod
    def set_imu_mode(limelight_name: str, mode: int) -> None:
        LimelightHelpers.set_limelight_nt_double(limelight_name, "imumode_set", mode)

    @staticmethod
    def set_fiducial_3d_offset(limelight_name: str, x: float, y: float, z: float) -> None:
        entries = [x, y, z]
        LimelightHelpers.set_limelight_nt_double_array(limelight_name, "fiducial_offset_set", entries)

    @staticmethod
    def set_fiducial_id_filters_override(limelight_name: str, valid_ids: List[int]) -> None:
        valid_ids_double = [float(id) for id in valid_ids]
        LimelightHelpers.set_limelight_nt_double_array(limelight_name, "fiducial_id_filters_set", valid_ids_double)

    @staticmethod
    def set_fiducial_downscaling_override(limelight_name: str, downscale: float) -> None:
        d = 0  # pipeline
        if downscale == 1.0:
            d = 1
        elif downscale == 1.5:
            d = 2
        elif downscale == 2.0:
            d = 3
        elif downscale == 3.0:
            d = 4
        elif downscale == 4.0:
            d = 5
        LimelightHelpers.set_limelight_nt_double(limelight_name, "fiducial_downscale_set", d)

    @staticmethod
    def set_camera_pose_robot_space(limelight_name: str, forward: float, side: float, up: float, roll: float, pitch: float, yaw: float) -> None:
        entries = [forward, side, up, roll, pitch, yaw]
        LimelightHelpers.set_limelight_nt_double_array(limelight_name, "camerapose_robotspace_set", entries)

    @staticmethod
    def set_python_script_data(limelight_name: str, outgoing_python_data: List[float]) -> None:
        LimelightHelpers.set_limelight_nt_double_array(limelight_name, "llrobot", outgoing_python_data)

    @staticmethod
    def get_python_script_data(limelight_name: str) -> List[float]:
        return LimelightHelpers.get_limelight_nt_double_array(limelight_name, "llpython")

    @staticmethod
    def take_snapshot(table_name: str, snapshot_name: str) -> bool:
        url = LimelightHelpers.get_limelight_url_string(table_name, "capturesnapshot")
        if url is None:
            return False
        try:
            response = requests.get(url, headers={"snapname": snapshot_name})
            return response.status_code == 200
        except requests.RequestException as e:
            print(f"Error taking snapshot: {e}")
            return False

    @staticmethod
    def get_latest_results(limelight_name: str) -> 'LimelightResults':
        start = time.time()
        results = LimelightHelpers.LimelightResults()
        try:
            json_dump = LimelightHelpers.get_json_dump(limelight_name)
            results = json.loads(json_dump, object_hook=lambda d: LimelightHelpers.LimelightResults(**d))
        except json.JSONDecodeError as e:
            results.error = f"lljson error: {e}"
        end = time.time()
        results.latency_json_parse = (end - start) * 1000  # Convert to milliseconds
        if LimelightHelpers.profile_json:
            print(f"lljson: {results.latency_json_parse:.2f} ms")
        return results

    class LimelightResults:
        def __init__(self, **kwargs):
            self.error = kwargs.get("error", "")
            self.pipeline_id = kwargs.get("pID", 0)
            self.latency_pipeline = kwargs.get("tl", 0)
            self.latency_capture = kwargs.get("cl", 0)
            self.latency_json_parse = kwargs.get("latency_jsonParse", 0)
            self.timestamp_limelight_publish = kwargs.get("ts", 0)
            self.timestamp_riofpga_capture = kwargs.get("ts_rio", 0)
            self.valid = kwargs.get("v", False)
            self.botpose = kwargs.get("botpose", [])
            self.botpose_wpired = kwargs.get("botpose_wpired", [])
            self.botpose_wpiblue = kwargs.get("botpose_wpiblue", [])
            self.botpose_tagcount = kwargs.get("botpose_tagcount", 0)
            self.botpose_span = kwargs.get("botpose_span", 0)
            self.botpose_avgdist = kwargs.get("botpose_avgdist", 0)
            self.botpose_avgarea = kwargs.get("botpose_avgarea", 0)
            self.camerapose_robotspace = kwargs.get("t6c_rs", [])
            self.targets_retro = [LimelightHelpers.LimelightTargetRetro(**t) for t in kwargs.get("Retro", [])]
            self.targets_fiducials = [LimelightHelpers.LimelightTargetFiducial(**t) for t in kwargs.get("Fiducial", [])]
            self.targets_classifier = [LimelightHelpers.LimelightTargetClassifier(**t) for t in kwargs.get("Classifier", [])]
            self.targets_detector = [LimelightHelpers.LimelightTargetDetector(**t) for t in kwargs.get("Detector", [])]
            self.targets_barcode = [LimelightHelpers.LimelightTargetBarcode(**t) for t in kwargs.get("Barcode", [])]

    class LimelightTargetRetro:
        def __init__(self, **kwargs):
            self.camera_pose_target_space = kwargs.get("t6c_ts", [])
            self.robot_pose_field_space = kwargs.get("t6r_fs", [])
            self.robot_pose_target_space = kwargs.get("t6r_ts", [])
            self.target_pose_camera_space = kwargs.get("t6t_cs", [])
            self.target_pose_robot_space = kwargs.get("t6t_rs", [])
            self.ta = kwargs.get("ta", 0)
            self.tx = kwargs.get("tx", 0)
            self.ty = kwargs.get("ty", 0)
            self.tx_pixels = kwargs.get("txp", 0)
            self.ty_pixels = kwargs.get("typ", 0)
            self.tx_nocrosshair = kwargs.get("tx_nocross", 0)
            self.ty_nocrosshair = kwargs.get("ty_nocross", 0)
            self.ts = kwargs.get("ts", 0)

    class LimelightTargetFiducial:
        def __init__(self, **kwargs):
            self.fiducial_id = kwargs.get("fID", 0)
            self.fiducial_family = kwargs.get("fam", "")
            self.camera_pose_target_space = kwargs.get("t6c_ts", [])
            self.robot_pose_field_space = kwargs.get("t6r_fs", [])
            self.robot_pose_target_space = kwargs.get("t6r_ts", [])
            self.target_pose_camera_space = kwargs.get("t6t_cs", [])
            self.target_pose_robot_space = kwargs.get("t6t_rs", [])
            self.ta = kwargs.get("ta", 0)
            self.tx = kwargs.get("tx", 0)
            self.ty = kwargs.get("ty", 0)
            self.tx_pixels = kwargs.get("txp", 0)
            self.ty_pixels = kwargs.get("typ", 0)
            self.tx_nocrosshair = kwargs.get("tx_nocross", 0)
            self.ty_nocrosshair = kwargs.get("ty_nocross", 0)
            self.ts = kwargs.get("ts", 0)

    class LimelightTargetBarcode:
        def __init__(self, **kwargs):
            self.family = kwargs.get("fam", "")
            self.data = kwargs.get("data", "")
            self.tx_pixels = kwargs.get("txp", 0)
            self.ty_pixels = kwargs.get("typ", 0)
            self.tx = kwargs.get("tx", 0)
            self.ty = kwargs.get("ty", 0)
            self.tx_nocrosshair = kwargs.get("tx_nocross", 0)
            self.ty_nocrosshair = kwargs.get("ty_nocross", 0)
            self.ta = kwargs.get("ta", 0)
            self.corners = kwargs.get("pts", [])

    class LimelightTargetClassifier:
        def __init__(self, **kwargs):
            self.class_name = kwargs.get("class", "")
            self.class_id = kwargs.get("classID", 0)
            self.confidence = kwargs.get("conf", 0)
            self.zone = kwargs.get("zone", 0)
            self.tx = kwargs.get("tx", 0)
            self.tx_pixels = kwargs.get("txp", 0)
            self.ty = kwargs.get("ty", 0)
            self.ty_pixels = kwargs.get("typ", 0)

    class LimelightTargetDetector:
        def __init__(self, **kwargs):
            self.class_name = kwargs.get("class", "")
            self.class_id = kwargs.get("classID", 0)
            self.confidence = kwargs.get("conf", 0)
            self.ta = kwargs.get("ta", 0)
            self.tx = kwargs.get("tx", 0)
            self.ty = kwargs.get("ty", 0)
            self.tx_pixels = kwargs.get("txp", 0)
            self.ty_pixels = kwargs.get("typ", 0)
            self.tx_nocrosshair = kwargs.get("tx_nocross", 0)
            self.ty_nocross = kwargs.get("ty_nocross", 0)

    class IMUData:
        def __init__(self, imu_data: List[float] = None):
            if imu_data and len(imu_data) >= 10:
                self.robot_yaw = imu_data[0]
                self.roll = imu_data[1]
                self.pitch = imu_data[2]
                self.yaw = imu_data[3]
                self.gyro_x = imu_data[4]
                self.gyro_y = imu_data[5]
                self.gyro_z = imu_data[6]
                self.accel_x = imu_data[7]
                self.accel_y = imu_data[8]
                self.accel_z = imu_data[9]
            else:
                self.robot_yaw = 0.0
                self.roll = 0.0
                self.pitch = 0.0
                self.yaw = 0.0
                self.gyro_x = 0.0
                self.gyro_y = 0.0
                self.gyro_z = 0.0
                self.accel_x = 0.0
                self.accel_y = 0.0
                self.accel_z = 0.0