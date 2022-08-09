package frc.robot.subsystem.drive.cmd;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.drive.DriveSubsystem2;

public class SlowDrive extends CommandBase {

    private final DriveSubsystem2 driveSubsystem;

    public SlowDrive(DriveSubsystem2 driveSubsystem) {
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
