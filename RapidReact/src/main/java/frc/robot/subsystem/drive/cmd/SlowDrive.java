package frc.robot.subsystem.drive.cmd;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.drive.DriveSubsystem;

public class SlowDrive extends CommandBase {

    private final DriveSubsystem driveSubsystem;

    public SlowDrive(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
    }

    @Override
    public void initialize() {
        driveSubsystem.speedModifier(0.75);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.speedModifier(1.00);
    }
}
