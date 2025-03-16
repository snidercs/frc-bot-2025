import threading
import logging
import requests
import limelight
import limelightresults
from wpimath.geometry import Pose2d, Translation2d, Rotation2d
from wpimath.units import degreesToRadians
from typing import Optional, List


class LimelightHelper:
    """
    LimelightHelper handles the initialization and periodic updates of the Limelight cameras.
    """

    def __init__(self):
        """
        Initialize the Limelight helper with automatic discovery.
        """
        self.limelight1: Optional[limelight.Limelight] = None
        self.limelight2: Optional[limelight.Limelight] = None
        self.addys: List[str] = []
        self.limelight1lastresult = None
        self.limelight2lastresult = None

        logging.critical("----------- LIMELIGHT HELPER INITIALIZED -----------")
        self._initialize_limelights()
        logging.critical("----------- LIMELIGHT HELPER FINISHED -----------")

    def _initialize_limelights(self) -> None:
        """
        Discover and initialize Limelight cameras.
        """
        self.addys = limelight.discover_limelights()

        if not self.addys:
            logging.warning("No Limelights found")
            return

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
        '''
        for i, limelight_instance in enumerate(limelights):
            if limelight_instance is not None:
                try:
                    limelight_instance.enable_websocket()
                    logging.info(f"Websocket enabled for Limelight{i + 1}")
                except Exception as e:
                    logging.error(f"Failed to enable websocket for Limelight{i + 1}: {e}")
        '''

    def get_primary_fiducial_id(self, limelight_instance: limelight.Limelight) -> Optional[int]:
        """
        Returns the ID of the primary in-view AprilTag as reported by the given Limelight,
        or -1 if no tag is detected.

        :param limelight_instance: The Limelight instance.
        :return: ID of the primary tag or -1 if none in view.
        """
        if limelight_instance:
            result = limelight_instance.get_results()
            parsed_result = limelightresults.parse_results(result)
            if parsed_result and parsed_result.fiducialResults:
                return parsed_result.fiducialResults[0].fiducial_id
        return None

    def get_pose2d(self, in_data: List) -> Optional[Pose2d]:
        """
        Converts an array of doubles to a Pose2d object.

        :param limelight_instance: The Limelight instance.
        :return: A Pose2d object or None if input is invalid.
        """
        if len(in_data) >= 6:
            tran2d = Translation2d(in_data[0], in_data[1])
            r2d = Rotation2d(degreesToRadians(in_data[5]))
            return Pose2d(tran2d, r2d)
        return None
    
    def get_robot_pose(self, limelight_instance: limelight.Limelight) -> Optional[Pose2d]:
        """
        Returns the robot pose in target
        """
        if limelight_instance:
            result = limelight_instance.get_results()
            parsed_result = limelightresults.parse_results(result)
            return self.get_pose2d(parsed_result.botpose)

    def get_target_pose(self, limelight_instance: limelight.Limelight) -> Optional[List[float]]:
        """
        Returns the target pose for the primary fiducial ID.

        :param limelight_instance: The Limelight instance.
        :return: The target pose or None if not found.
        """
        if limelight_instance:
            result = limelight_instance.get_results()
            parsed_result = limelightresults.parse_results(result)
            primary_id = self.get_primary_fiducial_id(limelight_instance)
            if parsed_result and parsed_result.fiducialResults:
                for fiducial_result in parsed_result.fiducialResults:
                    if fiducial_result.fiducial_id == primary_id:
                        return self.get_pose2d(fiducial_result.target_pose_robot_space)
        return None