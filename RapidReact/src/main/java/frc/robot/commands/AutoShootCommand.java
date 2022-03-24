package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystem.IntakeSubsystem;
import frc.robot.subsystem.RGBSubsystem;
import frc.robot.subsystem.ShooterSubsystem;

public class AutoShootCommand extends SequentialCommandGroup
{
    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private RGBSubsystem rgb;

    private boolean top;

    public AutoShootCommand(ShooterSubsystem shooter, IntakeSubsystem intake, RGBSubsystem rgb)
    {
        this.shooter = shooter;
        this.intake = intake;
        this.rgb = rgb;

        //Default Parameters
        this.withParameters(2, true);
    }

    public AutoShootCommand withParameters(int ballCount, boolean top)
    {
        this.top = top;

        this.addCommands(ballCount == 1 ? this.getSingleBallShootCommand() : this.getDoubleBallShootCommand());

        return this;
    }

    private Command getSingleBallShootCommand()
    {
        return new InstantCommand(this.top ? () -> this.shooter.spinUpTop() : () -> this.shooter.shootLow()) //Activate shooter
                .andThen(new WaitCommand(0.3) //new WaitUntilCommand(() -> this.shooter.isUpToSpeed()) //Wait until shooter is up to speed
                        .andThen(() -> this.rgb.autoShootingSingle()) //Very important RGB
                        .andThen(this::enableFeedersBMS) //Feed ball
                        .andThen(new WaitCommand(0.2) //Wait a bit, then turn off everything (how long it takes ball to shoot)
                                .andThen(this::disableShooterFeedersBMS)));
    }

    private Command getDoubleBallShootCommand()
    {
        return new InstantCommand(this.top ? () -> this.shooter.spinUpTop() : () -> this.shooter.shootLow()) //Activate shooter
                .andThen(new WaitUntilCommand(() -> this.shooter.isUpToHighSpeed()) //Wait until shooter is up to speed
                        .andThen(() -> this.rgb.autoShootingDouble()) //Very important RGB
                        .andThen(this::enableFeedersBMS) //Activate feeders (and BMS just in case)
                        .andThen(new WaitCommand(0.2) //Wait, then turn off feeders (how long ball #1 takes to shoot)
                                .andThen(this::disableFeedersBMS)
                                .andThen(new WaitUntilCommand(() -> this.shooter.isUpToHighSpeed()) //Wait for shooter to get up to speed again (pause between shots, shooting ball #2)
                                        .andThen(new WaitCommand(0.7) //Extra pause between shots
                                                .andThen(this::enableFeedersBMS)
                                                .andThen(new WaitCommand(0.5) //Wait, then turn off everything (how long it takes ball #2 to feed and shoot)
                                                        .andThen(this::disableShooterFeedersBMS))))));
    }

    private void enableFeedersBMS()
    {
        this.shooter.turnOnFeeders();
        this.intake.ballManagementForward();
    }

    private void disableFeedersBMS()
    {
        this.shooter.turnOffFeeders();
        this.intake.stopBallManagement();
    }

    private void disableShooterFeedersBMS()
    {
        this.shooter.stopShoot();
        this.shooter.turnOffFeeders();
        this.intake.stopBallManagement();
    }
}
