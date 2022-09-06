package frc.robot.drive.teleop;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.drive.DriveSystem;
import frc.robot.lib.fsm.StateHandler;
import frc.robot.lib.resources.b.prc;

import java.util.List;

public class LockHSystem implements StateHandler<TeleopState> {

    final prc<DriveSystem> driveSystem;

    public LockHSystem(prc<DriveSystem> driveSystem) {
        this.driveSystem = driveSystem;
    }

    @Override
    public List<TeleopState> thisHandles() {
        return List.of(TeleopState.LOCKED);
    }

    @Override
    public void onEvent() {
        DriveSystem system = driveSystem.tryAcquire();
        if (system == null) return;

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
