package frc.robot.lib.exec;

import java.util.function.BooleanSupplier;

public class MyPressable implements Pressable {

    private final BooleanSupplier button;

    boolean pressedLast;

    public MyPressable(BooleanSupplier button) {
        this.button = button;

        pressedLast = button.getAsBoolean();
    }

    @Override
    public boolean pressed() {
        boolean pressed = button.getAsBoolean();

        if (!pressedLast && pressed) {
            pressedLast = true;

            return true;
        } else {
            pressedLast = pressed;

            return false;
        }

    }

    @Override
    public boolean held() {
        return button.getAsBoolean();
    }

    @Override
    public boolean released() {
        boolean pressed = button.getAsBoolean();

        if (!pressedLast && pressed) {
            pressedLast = true;

            return true;
        } else {
            pressedLast = pressed;

            return false;
        }
    }
}
