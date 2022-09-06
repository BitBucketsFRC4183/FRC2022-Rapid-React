package frc.robot.drive.command;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.drive.DriveSystem;

public class DriveCommands {

    final DriveSystem drive;

    public DriveCommands(DriveSystem drive) {
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
