import commands2
import wpilib
import choreo

class AutonomousCommand(commands2.Command):
    def __init__(self, drivetrain, intake, traj, is_red_alliance):
        """
        Initializes the AutonomousCommand.

        :param drivetrain: The drivetrain subsystem used by this command.
        :param traj: The trajectory file to follow.
        :param is_red_alliance: Boolean indicating if the robot is on the red alliance.
        """
        super().__init__()
        self.drivetrain = drivetrain
        self.intake = intake
        self.trajectory = choreo.load_swerve_trajectory(traj)
        self.timer = wpilib.Timer()
        self.is_red_alliance = is_red_alliance
        self.addRequirements(self.drivetrain)  # Ensure the drivetrain is a requirement for this command
        self.laststamp = 0
        self.event_markers = []
        self.triggered_events = set()

    def initialize(self):
        """
        This autonomous runs the autonomous command selected by your RobotContainer class.
        """
        if self.trajectory:
            # Get the initial pose of the trajectory
            initial_pose = self.trajectory.get_initial_pose(self.is_red_alliance)

            if initial_pose:
                # Reset odometry to the start of the trajectory
                self.drivetrain.reset_pose(initial_pose)

            # Load event markers from the trajectory
            self.event_markers = self.trajectory.events

        # Reset and start the timer when the autonomous period begins
        self.timer.restart()

    def execute(self):
        """
        This function is called periodically during autonomous.
        """
        if self.trajectory:
            # Sample the trajectory at the current time into the autonomous period
            sample = self.trajectory.sample_at(self.timer.get(), self.is_red_alliance)

            if sample:
                if sample.timestamp != self.laststamp:
                    # Command the drivetrain to follow the sampled trajectory
                    self.drivetrain.follow_trajectory(sample)
                    self.laststamp = sample.timestamp

                    # Check for event markers and trigger actions
                    for marker in self.event_markers:
                        if marker.timestamp <= self.timer.get() < marker.timestamp + 0.2:
                            if marker.event not in self.triggered_events:
                                self.trigger_event(marker.event)
                                self.triggered_events.add(marker.event)

                else:
                    self.drivetrain.stop()

    def trigger_event(self, event):
        """
        Trigger the action associated with the event.
        """
        # Implement the actions to be triggered by the event
        if event == "CoralPlace":
            # Perform some action
            self.intake.setMotor(0.2)
        elif event == "CoralIntake":
            # Perform another action
            self.intake.setMotor(-0.2)

    def isFinished(self):
        """
        Returns true when the command should end.
        """
        return False