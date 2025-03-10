from commands2 import Subsystem
import logging
import requests
import limelight
import limelightresults
import ntcore

class LimelightSubsystem(Subsystem):
    """
    LimelightSubsystem handles the initialization and periodic updates of the Limelight cameras.
    """

    def __init__(self):
        super().__init__()
        self.limelight1 = None
        self.limelight2 = None
        self.addys = []

        # Limelight init
        self.limelight_init()

    def limelight_init(self) -> None:
        """
        Initialize the Limelight cameras by discovering and setting them up.
        """
        print("----------- LIMELIGHT -----------")
        self.addys = limelight.discover_limelights()
        if not self.addys:
            print("No Limelights found")
            self.limelight1 = None
            self.limelight2 = None
            return

        if len(self.addys) >= 1:
            self.limelight1 = limelight.Limelight(self.addys[0])
            try:
                result1 = self.limelight1.get_results()
                print(result1)
                parsed_result1 = limelightresults.parse_results(result1)
                for result in parsed_result1.fiducialResults:
                    print("Limelight1 fiducial_id")
                    print(result.fiducial_id)
            except requests.exceptions.ConnectionError as e:
                logging.error(f"Failed to connect to Limelight1: {e}")
                self.limelight1 = None
        else:
            self.limelight1 = None

        if len(self.addys) >= 2:
            self.limelight2 = limelight.Limelight(self.addys[1])
            try:
                result2 = self.limelight2.get_results()
                print(result2)
                parsed_result2 = limelightresults.parse_results(result2)
                for result in parsed_result2.fiducialResults:
                    print("Limelight2 fiducial_id")
                    print(result.fiducial_id)
            except requests.exceptions.ConnectionError as e:
                logging.error(f"Failed to connect to Limelight2: {e}")
                self.limelight2 = None
        else:
            self.limelight2 = None

        print("----------- LIMELIGHT -----------")

    def teleop_periodic(self) -> None:
        """
        Periodic updates for the Limelight cameras during teleop mode.
        """
        if self.limelight1 is not None:
            status1 = self.limelight1.get_status()
            result1 = self.limelight1.get_results()
            parsed_result1 = limelightresults.parse_results(result1)
            for fiducial_result in parsed_result1.fiducialResults:
                print(f"Limelight1 fiducial_id: {fiducial_result.fiducial_id}, cpu: {status1['cpu']}")

        if self.limelight2 is not None:
            status2 = self.limelight2.get_status()
            result2 = self.limelight2.get_results()
            parsed_result2 = limelightresults.parse_results(result2)
            for fiducial_result in parsed_result2.fiducialResults:
                print(f"Limelight2 fiducial_id: {fiducial_result.fiducial_id}, cpu: {status2['cpu']}")

    def get_primary_fiducial_id(self, limelight_name: str) -> int:
        """
        Returns the ID of the primary in-view AprilTag as reported by the given Limelight,
        or -1 if no tag is detected.

        :param limelight_name: The name of the Limelight NetworkTable ("limelight" if default).
        :return: ID of the primary tag or -1 if none in view.
        """
        limelight_table = ntcore.getTable(limelight_name)
        tag_id = limelight_table.getEntry("tid").getDouble(-1.0)
        return int(tag_id)

    def is_connected(self, limelight_name: str) -> bool:
        """
        Checks if the Limelight is currently sending JSON data via NetworkTables.
        Returns true if the Limelight appears to be connected and publishing,
        false otherwise.

        :param limelight_name: The name of the Limelight NetworkTable ("limelight" if default).
        :return: True if connected, False otherwise.
        """
        limelight_table = ntcore.getTable(limelight_name)
        latency = limelight_table.getEntry("tl").getDouble(0.0)
        return latency > 0.01
