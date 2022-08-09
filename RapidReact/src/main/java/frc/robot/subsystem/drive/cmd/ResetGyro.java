package frc.robot.subsystem.drive.cmd;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.drive.DriveSubsystem2;

public class ResetGyro extends CommandBase {

    private final DriveSubsystem2 driveSubsystem;

    public ResetGyro(DriveSubsystem2 driveSubsystem) {
        this.driveSubsystem = driveSubsystem;

        this.addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        driveSubsystem.resetOdometer();
    }
}
