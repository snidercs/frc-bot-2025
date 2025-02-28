import commands2
import wpilib
import choreo

class AutonomousCommand(commands2.Command):
    def __init__(self, drivetrain, traj, is_red_alliance):
        """
        Initializes the AutonomousCommand.

        :param drivetrain: The drivetrain subsystem used by this command.
        :param traj: The trajectory file to follow.
        :param is_red_alliance: Boolean indicating if the robot is on the red alliance.
        """
        super().__init__()
        self.drivetrain = drivetrain
        self.trajectory = choreo.load_swerve_trajectory(traj)
        self.timer = wpilib.Timer()
        self.is_red_alliance = is_red_alliance
        self.addRequirements(self.drivetrain)  # Ensure the drivetrain is a requirement for this command
        self.laststamp = 0

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
                print(f"Sampled trajectory at {sample.timestamp} seconds")
                print(f"laststamp: {self.laststamp}")
                if sample.timestamp != self.laststamp:

                    # Command the drivetrain to follow the sampled trajectory
                    self.drivetrain.follow_trajectory(sample)
                    self.laststamp = sample.timestamp
                else:
                    self.drivetrain.stop()

            
    def isFinished(self):
        """
        Returns true when the command should end.
        """
        return False