package frc.robot.drive.teleop;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.drive.DriveSystem;
import frc.robot.lib.fsm.StateSystem;
import frc.robot.lib.resources.b.prc;

public class TeleopLockSystem implements StateSystem<TeleopState> {

    final prc<DriveSystem> driveSystem;

    public TeleopLockSystem(prc<DriveSystem> driveSystem) {
        this.driveSystem = driveSystem;
    }

    @Override
    public void consume(TeleopState teleopState) {
        if (teleopState == TeleopState.LOCKED) {
            DriveSystem system = driveSystem.tryAcquire();

            if (system != null) {
                system.driveAt(
                        new SwerveModuleState[]{
                                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                                new SwerveModuleState(0, Rotation2d.fromDegrees(45))
                        }
                );
            }
        }
    }
}