package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.log.BucketLog;
import frc.robot.log.LogLevel;
import frc.robot.log.Loggable;
import frc.robot.log.Put;
import frc.robot.subsystem.*;
import frc.robot.utils.AutonomousPath;

import java.util.Optional;

public class AutonomousCommand extends SequentialCommandGroup
{
    private AutonomousSubsystem auto;
    private DrivetrainSubsystem drive;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private RGBSubsystem rgb;

    private final Loggable<String> state = BucketLog.loggable(Put.STRING, "auto/commandState");

    private Optional<Pose2d> initialPosition;

    public AutonomousCommand(AutonomousSubsystem auto, DrivetrainSubsystem drive, IntakeSubsystem intake, ShooterSubsystem shooter, RGBSubsystem rgb)
    {
        this.auto = auto;
        this.drive = drive;
        this.intake = intake;
        this.shooter = shooter;
        this.rgb = rgb;

        this.initialPosition = Optional.empty();
    }

    //Basic Commands
    public AutoShootCommand getShootCommand(int ballCount, boolean top)
    {
        return new AutoShootCommand(this.shooter, this.intake, this.rgb)
                .withParameters(ballCount, top);
    }

    public InstantCommand getDropIntakeCommand()
    {
        return this.actionToCommand((d, i, s) -> {
            i.forceIntaking();
            i.spinForward();
            s.antiFeed();
        });
    }

    public InstantCommand getSpinShooterCommand(boolean top)
    {
        return this.actionToCommand((d, i, s) -> {
            if(top) s.spinUpTop();
            else s.shootLow();
        });
    }

    public SequentialCommandGroup getSpinShooterCommandWithDelay(boolean top, double delay)
    {
        return new WaitCommand(delay).andThen(this.getSpinShooterCommand(top));
    }

    //AutonomousCommand Builders

    public AutonomousCommand shootOne(boolean top)
    {
        this.addCommands(
                new InstantCommand(() -> this.intake.stopSpin())
                .andThen(this.getShootCommand(1, top))
                .andThen(new InstantCommand(() -> this.intake.spinForward()))
        );
        return this;
    }

    public AutonomousCommand shootTwo(boolean top)
    {
        this.addCommands(
                new InstantCommand(() -> this.intake.stopSpin())
                .andThen(this.getShootCommand(2, top))
                .andThen(new InstantCommand(() -> {
                    this.intake.spinForward();
                    this.shooter.antiFeed();
                }))
        );
        return this;
    }

    public AutonomousCommand executeDrivePath(AutonomousPath path, double delay)
    {
        return this.executeDrivePath(path.pathName, delay, null);
    }

    public AutonomousCommand executeDrivePath(AutonomousPath path, double delay, Command parallel)
    {
        return this.executeDrivePath(path.pathName, delay, parallel);
    }

    public AutonomousCommand executeDrivePath(String pathPlanner, double delay)
    {
        return this.executeDrivePath(pathPlanner, delay, null);
    }

    public AutonomousCommand executeDrivePath(String pathPlanner, double delay, Command parallel)
    {
        PathPlannerTrajectory t = this.auto.buildPath(pathPlanner);

        SequentialCommandGroup drive = new WaitCommand(delay)
                .andThen(new AutonomousFollowPathCommand(t, this.auto, this.drive, this.rgb));

        if(parallel == null) this.addCommands(drive);
        else this.addCommands(drive.alongWith(parallel));

        if(this.initialPosition.isEmpty()) this.setInitialPosition(t);
        return this;
    }

    public AutonomousCommand executeAction(SubsystemAction action, double delay)
    {
        this.addCommands(new WaitCommand(delay)
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

    public AutonomousCommand stop()
    {
        this.addCommands(this.actionToCommand((d, i, s) -> {
            d.stopSticky();
            i.stopSpin();
            s.disable();
        }));

        //Set Odometry
        Pose2d zeroPos = new Pose2d(0, 0, new Rotation2d(0));
        if (this.initialPosition.isPresent()) {
            SmartDashboard.putString("/drivetrain/initial_path_position", this.initialPosition.get().toString());
            this.drive.resetGyroWithOffset(this.initialPosition.orElse(zeroPos).getRotation());
            this.drive.setOdometry(this.initialPosition.orElse(zeroPos));
        }
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
