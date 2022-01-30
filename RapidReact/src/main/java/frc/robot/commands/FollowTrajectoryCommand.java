package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystem.DrivetrainSubsystem;

public class FollowTrajectoryCommand extends InstantCommand
{
    private PathPlannerTrajectory trajectory;
    private DrivetrainSubsystem drive;

    public FollowTrajectoryCommand(String trajectoryPath, DrivetrainSubsystem drive)
    {
        this.trajectory = PathPlanner.loadPath(trajectoryPath, 5, 8);
        this.drive = drive;
    }

    @Override
    public void schedule()
    {
        this.drive.setOdometry(this.trajectory.getInitialPose());
        this.drive.field.setRobotPose(this.trajectory.getInitialPose());
        this.drive.gyro.setAngle(this.trajectory.getInitialPose().getRotation());
        this.drive.drivetrainModel.setKnownPose(this.trajectory.getInitialPose());

        this.drive.field.getObject("Trajectory").setTrajectory(this.trajectory);

        PPSwerveControllerCommand followPath = this.drive.drivetrainModel.createCommandForTrajectory(this.trajectory, this.drive, this.drive.kinematics);

        followPath.schedule();

        Pose2d actualEnd = this.drive.field.getRobotPose();
        Pose2d targetEnd = this.trajectory.getEndState().poseMeters;

        double distance = Math.pow(Math.pow(actualEnd.getX() - targetEnd.getX(), 2) + Math.pow(actualEnd.getY() - targetEnd.getY(), 2), 0.5);
        System.out.println("Distance from Path Target: " + distance);
    }

    public PathPlannerTrajectory getTrajectory()
    {
        return this.trajectory;
    }
}
