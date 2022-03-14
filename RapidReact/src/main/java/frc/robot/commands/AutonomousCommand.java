package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.log.BucketLog;
import frc.robot.log.LogLevel;
import frc.robot.log.Loggable;
import frc.robot.log.Put;
import frc.robot.subsystem.AutonomousSubsystem;
import frc.robot.subsystem.DrivetrainSubsystem;
import frc.robot.subsystem.IntakeSubsystem;
import frc.robot.subsystem.ShooterSubsystem;

import java.util.Optional;

public class AutonomousCommand extends SequentialCommandGroup
{
    private AutonomousSubsystem auto;
    private DrivetrainSubsystem drive;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;

    private final Loggable<String> state = BucketLog.loggable(Put.STRING, "auto/commandState");

    private Optional<Pose2d> initialPosition;

    public AutonomousCommand(AutonomousSubsystem auto, DrivetrainSubsystem drive, IntakeSubsystem intake, ShooterSubsystem shooter)
    {
        this.auto = auto;
        this.drive = drive;
        this.intake = intake;
        this.shooter = shooter;

        this.initialPosition = Optional.empty();
    }

    public AutonomousCommand shootPreload(boolean top)
    {
        InstantCommand shootCommand;
        if(top) shootCommand = new InstantCommand(() -> this.shooter.spinUpTop());
        else shootCommand = new InstantCommand(() -> this.shooter.shootLow());

        this.addCommands(shootCommand
                .andThen(new WaitCommand(1)
                        .andThen(() -> {
                            this.shooter.turnOnFeeders();
                            this.intake.ballManagementForward();
                        }).andThen(new WaitCommand(0.5)
                                .andThen(() -> {
                                    this.shooter.stopShoot();

                                    this.shooter.turnOffFeeders();
                                    this.intake.ballManagementBackward();
                                }))));

        return this;
    }

    public AutonomousCommand executeDrivePath(String pathPlanner)
    {
        PathPlannerTrajectory t = this.auto.buildPath(pathPlanner);
        this.addCommands(new AutonomousFollowPathCommand(t, this.auto, this.drive));

        if(this.initialPosition.isEmpty()) this.setInitialPosition(t);
        return this;
    }

    public AutonomousCommand executeDrivePath(String pathPlanner, double delayBeforeStart)
    {
        PathPlannerTrajectory t = this.auto.buildPath(pathPlanner);
        this.addCommands(new WaitCommand(delayBeforeStart)
                .andThen(new AutonomousFollowPathCommand(t, this.auto, this.drive)));

        if(this.initialPosition.isEmpty()) this.setInitialPosition(t);
        return this;
    }

    public AutonomousCommand executeAction(SubsystemAction action)
    {
        this.addCommands(this.actionToCommand(action));
        return this;
    }

    public AutonomousCommand executeAction(SubsystemAction action, double delayBeforeStart)
    {
        this.addCommands(new WaitCommand(delayBeforeStart)
                .andThen(this.actionToCommand(action)));
        return this;
    }

    private void setInitialPosition(PathPlannerTrajectory trajectory)
    {
        PathPlannerTrajectory.PathPlannerState state = trajectory.getInitialState();

        this.initialPosition = Optional.of(new Pose2d(
                state.poseMeters.getTranslation(),
                state.holonomicRotation
        ));
    }

    public AutonomousCommand complete()
    {
        this.addCommands(this.actionToCommand((d, i, s) -> {
            d.stop();
            i.stopSpin();
            s.disable();
        }));

        //Set Odometry
        Pose2d zeroPos = new Pose2d(0, 0, new Rotation2d(0));
        this.drive.resetGyroWithOffset(this.initialPosition.orElse(zeroPos).getRotation());
        this.drive.setOdometry(this.initialPosition.orElse(zeroPos));

        return this;
    }

    private InstantCommand actionToCommand(SubsystemAction action)
    {
        return new InstantCommand(() -> action.doAction(this.drive, this.intake, this.shooter));
    }

    @Override
    public void schedule()
    {
        this.state.log(LogLevel.GENERAL, "Autonomous Command Starting!");
        super.schedule();
        this.state.log(LogLevel.GENERAL, "Autonomous Command Completed!");
    }

    public interface SubsystemAction
    {
        void doAction(DrivetrainSubsystem drive, IntakeSubsystem intake, ShooterSubsystem shooter);
    }
}
