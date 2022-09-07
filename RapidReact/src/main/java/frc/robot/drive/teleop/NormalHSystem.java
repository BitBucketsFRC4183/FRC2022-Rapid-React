package frc.robot.drive.teleop;

import frc.robot.drive.DriveSystem;
import frc.robot.drive.input.DriveInputSystem;
import frc.robot.drive.odometry.OdometrySystem;
import frc.robot.lib.fsm.StateHandler;
import frc.robot.lib.resources.b.prc;

import java.util.List;

public class NormalHSystem implements StateHandler<TeleopState> {

    final prc<DriveSystem> driveSystem;
    final prc<DriveInputSystem> inputSystem;
    final prc<OdometrySystem> odometrySystem;

    public NormalHSystem(prc<DriveSystem> driveSystem, prc<DriveInputSystem> inputSystem, prc<OdometrySystem> odometrySystem) {
        this.driveSystem = driveSystem;
        this.inputSystem = inputSystem;
        this.odometrySystem = odometrySystem;
    }

    @Override
    public List<TeleopState> thisHandles() {
        return List.of(TeleopState.NORMAL);
    }

    @Override
    public void onEvent() {
        DriveSystem driveSystem = this.driveSystem.tryAcquire(NormalHSystem.class);
        DriveInputSystem inputSystem = this.inputSystem.tryParallel(NormalHSystem.class);
        OdometrySystem odometrySystem = this.odometrySystem.tryParallel(NormalHSystem.class);

        if (driveSystem == null) return;

        odometrySystem.calculateFieldOrientedDesiredStates(
                inputSystem.getSlewX(), inputSystem.getSlewY(), inputSystem.getRotation()
        );
    }
}
