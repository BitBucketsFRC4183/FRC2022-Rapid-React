package frc.robot.subsystem.drive.cmd;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.drive.DriveSubsystem;

public class ResetGyro extends CommandBase {

    private final DriveSubsystem driveSubsystem;

    public ResetGyro(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;

        this.addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        driveSubsystem.resetOdometer();
    }
}
