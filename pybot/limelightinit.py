from commands2 import Subsystem
import logging
import requests
import limelight
import limelightresults
import time

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

        if len(self.addys) >= 1:
            self.limelight1 = limelight.Limelight(self.addys[0])
            try:
                logging.info(f"Limelight alias {self.limelight1.get_name()} successfully connected")
            except (requests.exceptions.ConnectionError, Exception) as e:
                logging.error(f"Failed to connect to Limelight1: {e}")
                self.limelight1 = None
        else:
            self.limelight1 = None

        if len(self.addys) >= 2:
            self.limelight2 = limelight.Limelight(self.addys[1])
            try:
                logging.info(f"Limelight alias {self.limelight2.get_name()} successfully connected")
            except (requests.exceptions.ConnectionError, Exception) as e:
                logging.error(f"Failed to connect to Limelight2: {e}")
                self.limelight2 = None
        else:
            self.limelight2 = None

        # Enable websocket for limelight1
        if self.limelight1 is not None:
            self.limelight1.enable_websocket()

        # Enable websocket for limelight2
        if self.limelight2 is not None:
            self.limelight2.enable_websocket()

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
