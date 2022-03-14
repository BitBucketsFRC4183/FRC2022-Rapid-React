package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.config.Config;
import frc.robot.log.BucketLog;
import frc.robot.log.LogLevel;
import frc.robot.log.Loggable;
import frc.robot.log.Put;
import frc.robot.subsystem.AutonomousSubsystem;
import frc.robot.subsystem.DrivetrainSubsystem;

public class AutonomousFollowPathCommand extends SequentialCommandGroup
{
    private final PathPlannerTrajectory trajectory;
    private AutonomousSubsystem auto;
    private DrivetrainSubsystem drive;
    private Config.AutonomousConfig autoConfig;

    private final Loggable<String> state = BucketLog.loggable(Put.STRING, "auto/followPathState");

    public AutonomousFollowPathCommand(String trajectoryPath, AutonomousSubsystem auto, DrivetrainSubsystem drive)
    {
        this.autoConfig = new Config().auto;

        this.trajectory = trajectoryPath.equals(this.autoConfig.nothingPath) ? PathPlanner.loadPath(trajectoryPath, 0, 0) : PathPlanner.loadPath(trajectoryPath, this.autoConfig.maxPathFollowVelocity, this.autoConfig.maxPathFollowAcceleration);
        this.auto = auto;
        this.drive = drive;

        if(!this.auto.isGyroReset())
        {
            this.drive.resetGyroWithOffset(this.trajectory.getInitialPose().getRotation());
            this.auto.setGyroReset();
        }

        Pose2d start = new Pose2d(this.trajectory.getInitialState().poseMeters.getTranslation(), this.trajectory.getInitialState().holonomicRotation);
        this.drive.setOdometry(start);

        this.addCommands(this.setup(), this.createTrajectoryFollowerCommand());
    }

    private PPSwerveControllerCommand createTrajectoryFollowerCommand()
    {
        PIDController xyController = new PIDController(1, 0.1, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(this.autoConfig.maxPathFollowVelocity, this.autoConfig.maxPathFollowAcceleration));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        return new PPSwerveControllerCommand(
                this.trajectory, //Trajectory
                () -> this.drive.odometry.getPoseMeters(), //Robot Pose supplier
                this.drive.kinematics, //Swerve Drive Kinematics
                xyController, //PID Controller: X
                xyController, //PID Controller: Y
                thetaController, //PID Controller: Θ
                this.drive::setStates, //SwerveModuleState setter
                this.drive
        );
    }

    private InstantCommand setup()
    {
        return new InstantCommand(() ->{
            this.state.log(LogLevel.GENERAL, "Starting to Follow a Trajectory!");

            this.drive.zeroStates(this.trajectory.getInitialPose());
        });
    }
}
