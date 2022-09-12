package frc.robot.drive.command;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.drive.DriveConstSystem;

public class DriveCommands {

    final DriveConstSystem drive;

    public DriveCommands(DriveConstSystem drive) {
        this.drive = drive;
    }

    public void lock() {
        drive.driveAt(
                new SwerveModuleState[]{
                        new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                        new SwerveModuleState(0, Rotation2d.fromDegrees(45))
                }
        );
    }
}
