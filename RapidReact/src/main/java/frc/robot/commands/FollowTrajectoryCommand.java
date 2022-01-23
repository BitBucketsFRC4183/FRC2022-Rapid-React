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
        new PPSwerveControllerCommand(
                this.trajectory,
                () -> this.trajectory.getInitialPose(),
                drive.kinematics,
                new PIDController(0.002, 0.0005, -0.0001),
                new PIDController(0.002, 0.0005, -0.0001),
                new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(drive.maxVelocity_metersPerSecond, drive.maxAngularVelocity_radiansPerSecond)),
                drive::setStates,
                drive
        ).schedule();

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
