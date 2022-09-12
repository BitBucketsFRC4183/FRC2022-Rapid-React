package frc.robot.drive.teleop;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.log.DriveConst;
import frc.robot.lib.capability.BorrowValue;
import frc.robot.lib.fsm.StateHandler;

import java.util.List;

public class LockHSystem implements StateHandler<TeleopState> {

    final BorrowValue<DriveConst, DriveConst.Modify> driveSystem;

    public LockHSystem(BorrowValue<DriveConst, DriveConst.Modify> driveSystem) {
        this.driveSystem = driveSystem;
    }

    @Override
    public List<TeleopState> thisHandles() {
        return List.of(TeleopState.LOCKED);
    }

    @Override
    public void onEvent() {
        driveSystem.readModify("driving").driveAt(
                new SwerveModuleState[]{
                        new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                        new SwerveModuleState(0, Rotation2d.fromDegrees(45))
                }
        );
    }
}
