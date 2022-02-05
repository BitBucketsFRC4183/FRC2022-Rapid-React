package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.config.Config;
import frc.robot.log.LogLevel;
import frc.robot.subsystem.AutonomousSubsystem;
import frc.robot.subsystem.DrivetrainSubsystem;

public class AutonomousFollowPathCommand extends SequentialCommandGroup
{
    private final PathPlannerTrajectory trajectory;
    private AutonomousSubsystem auto;
    private DrivetrainSubsystem drive;

    public AutonomousFollowPathCommand(String trajectoryPath, AutonomousSubsystem auto, DrivetrainSubsystem drive)
    {
        this.trajectory = trajectoryPath.equals(new Config().auto.nothingPath) ? PathPlanner.loadPath(trajectoryPath, 0, 0) : PathPlanner.loadPath(trajectoryPath, 5, 8);
        this.auto = auto;
        this.drive = drive;

        this.addCommands(
                this.setup(),

                this.drive.drivetrainModel.createCommandForTrajectory(this.trajectory, this.drive, this.drive.kinematics),

                this.printError()
        );
    }

    private InstantCommand setup()
    {
        return new InstantCommand(() ->{
            this.drive.logger().logString(LogLevel.GENERAL, "auto/followState", "Starting to Follow a Trajectory!");
            this.drive.drivetrainModel.resetPID(this.trajectory.getInitialPose().getRotation().getRadians());
            this.drive.setOdometry(this.trajectory.getInitialPose());
            this.drive.field.setRobotPose(this.trajectory.getInitialPose());
            //this.drive.gyro.setAngle(this.trajectory.getInitialPose().getRotation());
            //this.drive.drivetrainModel.setKnownPose(this.trajectory.getInitialPose());

            this.drive.field.getObject("Trajectory").setTrajectory(this.trajectory);

            this.drive.zeroStates(this.trajectory.getInitialPose());
        });
    }

    private InstantCommand printError()
    {
        return new InstantCommand(() -> {
            Pose2d actualEnd = this.drive.field.getRobotPose();
            Pose2d targetEnd = this.trajectory.getEndState().poseMeters;

            double distance = Math.pow(Math.pow(actualEnd.getX() - targetEnd.getX(), 2) + Math.pow(actualEnd.getY() - targetEnd.getY(), 2), 0.5);
            this.drive.logger().logString(LogLevel.GENERAL, "auto/followState", "Completed following a Trajectory! Distance Error from Target: " + distance);
        });
    }
}
