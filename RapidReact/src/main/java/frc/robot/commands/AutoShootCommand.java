package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystem.IntakeSubsystem;
import frc.robot.subsystem.RGBSubsystem;
import frc.robot.subsystem.ShooterSubsystem;

public class AutoShootCommand extends InstantCommand
{
    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private RGBSubsystem rgb;

    private int ballCount;
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
        this.ballCount = ballCount;
        this.top = top;

        return this;
    }

    @Override
    public void schedule()
    {
        (this.ballCount == 1 ? this.getSingleBallShootCommand() : this.getDoubleBallShootCommand()).schedule();
    }

    private Command getSingleBallShootCommand()
    {
        return new InstantCommand(this.top ? () -> this.shooter.spinUpTop() : () -> this.shooter.shootLow()) //Activate shooter
                .andThen(new WaitUntilCommand(() -> this.shooter.isUpToSpeed()) //Wait until shooter is up to speed
                .andThen(() -> this.rgb.autoShootingSingle()) //Very important RGB
                .andThen(() -> { //Activate feeders (and BMS just in case)
                    this.shooter.turnOnFeeders();
                    this.intake.ballManagementForward();
                })
                .andThen(new WaitCommand(0.2) //Wait a bit
                .andThen(this::disableShooterFeedersBMS)));
    }

    private Command getDoubleBallShootCommand()
    {
        return new InstantCommand(this.top ? () -> this.shooter.spinUpTop() : () -> this.shooter.shootLow()) //Activate shooter
                .andThen(new WaitUntilCommand(() -> this.shooter.isUpToSpeed()) //Wait until shooter is up to speed
                .andThen(() -> this.rgb.autoShootingDouble()) //Very important RGB
                .andThen(() -> { //Activate feeders (and BMS just in case)
                    this.shooter.turnOnFeeders();
                    this.intake.ballManagementForward();
                })
                .andThen(new WaitCommand(0.2) //Wait, then turn off feeders (pause between shots)
                .andThen(() -> {
                    this.shooter.turnOffFeeders();
                    this.intake.stopBallManagement();
                })
                .andThen(new WaitCommand(0.2) //Wait, then turn the feeders back on (ball #2)
                .andThen(() -> {
                    this.shooter.turnOnFeeders();
                    this.intake.ballManagementForward();
                }))
                .andThen(new WaitCommand(0.2) //Wait, then turn off everything
                .andThen(this::disableShooterFeedersBMS))));
    }

    private void disableShooterFeedersBMS()
    {
        this.shooter.stopShoot();
        this.shooter.turnOffFeeders();
        this.intake.stopBallManagement();
    }
}
