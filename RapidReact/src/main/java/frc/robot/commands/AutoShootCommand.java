package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystem.other.IntakeSubsystem;
import frc.robot.subsystem.other.ShooterSubsystem;

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

                new InstantCommand(() -> this.shooter.isAutoShooting = true),

                new InstantCommand(() -> {
                    this.intake.ballManagementForward();
                    this.shooter.antiFeed();
                })
                        .andThen(new WaitCommand(0.5)
                                        .andThen(() -> {
                                            this.intake.stopBallManagement();
                                            this.shooter.turnOffFeeders();
                                        })),

                //Activate shooter
                new InstantCommand(this.top ? this.shooter::spinUpTop : this.shooter::shootLow),

                //Wait for shooter to get up to speed
                new WaitUntilCommand(this.top ? this.shooter::isUpToHighSpeed : this.shooter::isUpToLowSpeed)
                        /*.raceWith(new WaitCommand(0.5))*/,

                //Extremely important RGB
                new InstantCommand(this.rgb::autoShootingSingle),

                //Activate feeders for ball #1
                new InstantCommand(this::enableFeedersBMS),

                //Wait for ball #1 to shoot
                new WaitCommand(0.3),

                //Turn off everything
                new InstantCommand(this::disableShooterFeedersBMS),

                new InstantCommand(() -> {
                    this.shooter.isAutoShooting = false;
                    this.rgb.normalize();
                })
        );

        return command.raceWith(new WaitCommand(3).andThen(this::disableShooterFeedersBMS));
    }

    private Command getDoubleBallShootCommand()
    {
        Command antifeedStuff = new InstantCommand(() -> {
           shooter.antiFeed();
           //intake.ballManagementForward();
        }).raceWith(new WaitCommand(0.2));

        SequentialCommandGroup command = new SequentialCommandGroup();

        command.addCommands(
                new InstantCommand(() -> this.shooter.isAutoShooting = true),

                new InstantCommand(() -> {
                    this.intake.ballManagementForward();
                    this.shooter.antiFeed();
                })
                        .andThen(new WaitCommand(0.5)
                        .andThen(() -> {
                            this.intake.stopBallManagement();
                            this.shooter.turnOffFeeders();
                        })),

                //Activate shooter
                new InstantCommand(this.top ? this.shooter::spinUpTop : this.shooter::shootLow),

                //Wait until shooter is up to speed
                new WaitUntilCommand(this.top ? this.shooter::isUpToHighSpeed : this.shooter::isUpToLowSpeed)
                        /*.raceWith(new WaitCommand(0.5))*/,

                //Extremely important RGB
                new InstantCommand(this.rgb::autoShootingDouble),



                //Activate feeders
                new InstantCommand(this::enableFeedersBMS),

                //Wait for ball #1 to shoot
                new WaitCommand(0.1),

                //Turn off feeders (ball #1 is finished shooting)
                new InstantCommand((() -> {
                    this.disableFeedersBMS();
                    this.shooter.upToSpeedCount = 0;
                })),

                //now
                //Wait until shooter is up to speed again
                new WaitUntilCommand(this.top ? this.shooter::isUpToHighSpeed : this.shooter::isUpToLowSpeed)
                        //.alongWith(new WaitCommand(0.3))
                        //.alongWith(antifeedStuff)
                        .alongWith(new WaitCommand(0.3).andThen(antifeedStuff))
                       /* .raceWith(new WaitCommand(0.5))*/,

                //Extra pause between shots
                new WaitCommand(0.2),

                //Activate feeders for ball #2
                new InstantCommand(this::enableFeedersBMS),

                //Wait for ball #2 to shoot (takes longer because it travels farther to the shooter)
                new WaitCommand(0.3),

                //Turn off everything
                new InstantCommand(this::disableShooterFeedersBMS),

                new InstantCommand(() -> {
                    this.shooter.isAutoShooting = false;
                    this.rgb.normalize();
                })
        );

        return command.raceWith(new WaitCommand(5).andThen(this::disableShooterFeedersBMS));
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
        this.intake.ballManagementFeed();
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
