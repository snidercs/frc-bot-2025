from commands2 import Subsystem
import logging
import requests
import limelight
import limelightresults
# import time
from wpimath.geometry import Pose2d, Translation2d, Rotation2d
from wpimath.units import degreesToRadians

class LimelightSubsystem(Subsystem):
    """
    LimelightSubsystem handles the initialization and periodic updates of the Limelight cameras.
    """

    def __init__(self):
        super().__init__()
        self.limelight1 = None
        self.limelight2 = None
        self.addys = []
        self.limelight1lastresult = None
        self.limelight2lastresult = None

        # Limelight init
        self.limelight_init()

    def limelight_init(self) -> None:
        """
        Initialize the Limelight cameras by discovering and setting them up.
        """
        logging.info("----------- LIMELIGHT -----------")
        self.addys = limelight.discover_limelights()

        if not self.addys:
            logging.warning("No Limelights found")
            self.limelight1 = None
            self.limelight2 = None
            return

        # Initialize Limelights in a loop to avoid redundant code
        limelights = []
        for i, address in enumerate(self.addys[:2]):  # Limit to 2 Limelights
            try:
                limelight_instance = limelight.Limelight(address)
                logging.info(f"Limelight alias {limelight_instance.get_name()} successfully connected")
                limelights.append(limelight_instance)
            except (requests.exceptions.ConnectionError, Exception) as e:
                logging.error(f"Failed to connect to Limelight{i + 1}: {e}")
                limelights.append(None)

        # Assign Limelight instances
        self.limelight1 = limelights[0] if len(limelights) > 0 else None
        self.limelight2 = limelights[1] if len(limelights) > 1 else None

        # Enable websockets for connected Limelights
        for i, limelight_instance in enumerate(limelights):
            if limelight_instance is not None:
                try:
                    limelight_instance.enable_websocket()
                    logging.info(f"Websocket enabled for Limelight{i + 1}")
                except Exception as e:
                    logging.error(f"Failed to enable websocket for Limelight{i + 1}: {e}")

        logging.info("----------- LIMELIGHT -----------")

    def periodic(self) -> None:
        """
        Periodic updates for the Limelight cameras during teleop mode.
        """
        if self.limelight1 is not None:
            result1 = self.limelight1.get_results()
            self.limelight1lastresult = limelightresults.parse_results(result1)

        if self.limelight2 is not None:
            result2 = self.limelight2.get_results()
            self.limelight2lastresult = limelightresults.parse_results(result2)

    def get_primary_fiducial_id(self, limelight_instance) -> int:
        """
        Returns the ID of the primary in-view AprilTag as reported by the given Limelight,
        or -1 if no tag is detected.

        :param limelight_instance: The Limelight instance.
        :return: ID of the primary tag or -1 if none in view.
        """
        result = limelight_instance.get_results()
        parsed_result = limelightresults.parse_results(result)
        if parsed_result and parsed_result.fiducialResults:
            return parsed_result.fiducialResults[0].fiducial_id
        return -1

    def to_pose2d(self, in_data):
        """
        Converts an array of doubles to a Pose2d object.

        :param in_data: A list of doubles representing pose data.
        :return: A Pose2d object or a default Pose2d if input is invalid.
        """
        if len(in_data) < 6:
            logging.error("Bad LL 2D Pose Data!")
            return Pose2d()  # Return a default Pose2d object

        tran2d = Translation2d(in_data[0], in_data[1])
        r2d = Rotation2d(degreesToRadians(in_data[5]+180))
        return Pose2d(tran2d, r2d)

    def get_target_pose(self, fiducial_id: int):
        """
        Returns the target pose for the given fiducial ID.
        
        :param fiducial_id: The ID of the fiducial.
        :return: The target pose or None if not found.
        """
        if self.limelight1 is not None:
            result1 = self.limelight1.get_results()
            parsed_result1 = limelightresults.parse_results(result1)
            for fiducial_result in parsed_result1.fiducialResults:
                if fiducial_result.fiducial_id == fiducial_id:
                    return fiducial_result.target_pose_camera_space

        if self.limelight2 is not None:
            result2 = self.limelight2.get_results()
            parsed_result2 = limelightresults.parse_results(result2)
            for fiducial_result in parsed_result2.fiducialResults:
                if fiducial_result.fiducial_id == fiducial_id:
                    return fiducial_result.target_pose_camera_space

        return None
