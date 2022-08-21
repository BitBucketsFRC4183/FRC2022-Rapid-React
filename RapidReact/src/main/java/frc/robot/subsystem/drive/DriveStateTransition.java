package frc.robot.subsystem.drive;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.Buttons;
import frc.robot.lib.fsm.StateTransition;

public class DriveStateTransition implements StateTransition<DriveState> {

    private final DriveSystem system;

    public DriveStateTransition(DriveSystem system) {
        this.system = system;
    }

    @Override
    public DriveState parse(DriveState current) {
        return switch (current) {
            case LOCKED -> null;
            case NORMAL -> normal();
            case SLOW -> null;
        };
    }

    private final Joystick joystick = new Joystick(0);
    private final Button button = new Buttons().autoClimbStopLeft;

    DriveState normal() {

        if (button.get()) {
            system.slowMode();
            return DriveState.LOCKED;
        }


        return DriveState.NORMAL; //desired state
    }

    DriveState lock() {



        return DriveState.LOCKED;
    }
}
