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
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.config.PID;
import frc.robot.subsystem.DrivetrainSubsystem;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.stream.IntStream;

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

        PIDController xy = new PIDController(0.0001, -0.0001, -0.0001);
        ProfiledPIDController t = new ProfiledPIDController(0.0001, 0.0001, 0.0001, new TrapezoidProfile.Constraints(drive.maxVelocity_metersPerSecond, drive.maxAngularVelocity_radiansPerSecond));

//        PPSwerveControllerCommand followPath = new PPSwerveControllerCommand(
//                this.trajectory,
//                () -> this.drive.field.getRobotPose(),
//                drive.kinematics,
//                xy,
//                xy,
//                t,
//                drive::setStates,
//                drive
//        );

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
