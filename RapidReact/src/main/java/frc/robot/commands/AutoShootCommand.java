package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystem.IntakeSubsystem;
import frc.robot.subsystem.RGBSubsystem;
import frc.robot.subsystem.ShooterSubsystem;

import java.util.function.BooleanSupplier;

public class AutoShootCommand extends SequentialCommandGroup
{
    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;
    private final RGBSubsystem rgb;

    private boolean top;

    public AutoShootCommand(ShooterSubsystem shooter, IntakeSubsystem intake, RGBSubsystem rgb)
    {
        this.shooter = shooter;
        this.intake = intake;
        this.rgb = rgb;
    }

    public AutoShootCommand withParameters(int ballCount, boolean top)
    {
        this.top = top;

        this.addCommands(ballCount == 1 ? this.getSingleBallShootCommand() : this.getDoubleBallShootCommand());

        return this;
    }

    private Command getSingleBallShootCommand()
    {
        SequentialCommandGroup command = new SequentialCommandGroup();

        command.addCommands(

                //Activate shooter
                new InstantCommand(this.top ? this.shooter::spinUpTop : this.shooter::shootLow),

                //Wait for shooter to get up to speed
                new WaitUntilCommand(this.shooter::isUpToSpeed),

                //Extremely important RGB
                new InstantCommand(this.rgb::autoShootingSingle),

                //Activate feeders for ball #1
                new InstantCommand(this::enableFeedersBMS),

                //Wait for ball #1 to shoot
                new WaitCommand(0.2),

                //Turn off everything
                new InstantCommand(this::disableShooterFeedersBMS)
        );

        return command;
    }

    private Command getDoubleBallShootCommand()
    {
        SequentialCommandGroup command = new SequentialCommandGroup();

        command.addCommands(

                //Activate shooter
                new InstantCommand(this.top ? this.shooter::spinUpTop : this.shooter::shootLow),

                //Wait until shooter is up to speed
                new WaitUntilCommand(this.top ? this.shooter::isUpToHighSpeed : this.shooter::isUpToLowSpeed),

                //Extremely important RGB
                new InstantCommand(this.rgb::autoShootingDouble),

                //Activate feeders
                new InstantCommand(this::enableFeedersBMS),

                //Wait for ball #1 to shoot
                new WaitCommand(0.2),

                //Turn off feeders (ball #1 is finished shooting)
                new InstantCommand(this::disableFeedersBMS),

                //Wait until shooter is up to speed again
                new WaitUntilCommand(this.top ? this.shooter::isUpToHighSpeed : this.shooter::isUpToLowSpeed),

                //Extra pause between shots
                new WaitCommand(0.7),

                //Activate feeders for ball #2
                new InstantCommand(this::enableFeedersBMS),

                //Wait for ball #2 to shoot (takes longer because it travels farther to the shooter)
                new WaitCommand(0.5),

                //Turn off everything
                new InstantCommand(this::disableShooterFeedersBMS)
        );

        return command;
    }

    @Override
    public void cancel()
    {
        this.disableShooterFeedersBMS();

        super.cancel();
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
