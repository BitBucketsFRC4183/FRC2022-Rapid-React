package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.log.BucketLog;
import frc.robot.log.LogLevel;
import frc.robot.log.Loggable;
import frc.robot.log.Put;
import frc.robot.subsystem.AutonomousSubsystem;
import frc.robot.subsystem.DrivetrainSubsystem;
import frc.robot.subsystem.IntakeSubsystem;
import frc.robot.subsystem.ShooterSubsystem;

public class AutonomousCommand extends SequentialCommandGroup
{
    private AutonomousSubsystem auto;
    private DrivetrainSubsystem drive;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;

    private final Loggable<String> state = BucketLog.loggable(Put.STRING, "auto/commandState");

    public AutonomousCommand(AutonomousSubsystem auto, DrivetrainSubsystem drive, IntakeSubsystem intake, ShooterSubsystem shooter)
    {
        this.auto = auto;
        this.drive = drive;
        this.intake = intake;
        this.shooter = shooter;
    }

    public AutonomousCommand executeShootPreload()
    {
        this.addCommands(new InstantCommand(() -> this.shooter.shootTop())
                .andThen(new WaitUntilCommand(() -> this.shooter.isUpToSpeed())
                        .andThen(new WaitCommand(5)
                                .andThen(() -> this.shooter.stopShoot()))));
        return this;
    }

    public AutonomousCommand executeDrivePath(String pathPlanner)
    {
        this.addCommands(new AutonomousFollowPathCommand(pathPlanner, this.auto, this.drive));
        return this;
    }

    public AutonomousCommand executeDrivePath(String pathPlanner, double delayBeforeStart)
    {
        this.addCommands(new WaitCommand(delayBeforeStart)
                .andThen(new AutonomousFollowPathCommand(pathPlanner, this.auto, this.drive)));
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

    public AutonomousCommand executeParallel(String pathPlanner, SubsystemAction action)
    {
        this.addCommands(
                new AutonomousFollowPathCommand(pathPlanner, this.auto, this.drive)
                        .alongWith(this.actionToCommand(action)));
        return this;
    }

    public AutonomousCommand executeParallel(String pathPlanner, SubsystemAction action, double delayBeforeStart)
    {
        this.addCommands(new WaitCommand(delayBeforeStart)
                        .andThen(
                                new AutonomousFollowPathCommand(pathPlanner, this.auto, this.drive)
                                        .alongWith(this.actionToCommand(action))));
        return this;
    }

    public AutonomousCommand complete()
    {
        this.addCommands(this.actionToCommand((d, i, s) -> {
            d.stop();
            i.stopSpin();
            s.disable();
        }));
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
