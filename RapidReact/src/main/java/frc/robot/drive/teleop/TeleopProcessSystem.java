package frc.robot.drive.teleop;

import frc.robot.drive.input.DriveInputSystem;
import frc.robot.lib.fsm.StateProcessor;
import frc.robot.lib.resources.b.prc;

public class TeleopProcessSystem implements StateProcessor<TeleopState> {

    final prc<DriveInputSystem> inputSystem;

    public TeleopProcessSystem(prc<DriveInputSystem> inputSystem) {
        this.inputSystem = inputSystem;
    }

    @Override
    public TeleopState defaultState() {
        return TeleopState.NORMAL;
    }

    @Override
    public TeleopState evaluateNextState(TeleopState currentState) {
        DriveInputSystem system = inputSystem.tryParallel(DriveInputSystem.class);

        double deadbandX = system.normalizedX();
        double deadbandY = system.normalizedY();
        double deadbandRot = system.rotation();

        if (deadbandX == 0 && deadbandY == 0 && deadbandRot == 0) {
            return TeleopState.LOCKED;
        }

        return TeleopState.NORMAL;
    }
}
