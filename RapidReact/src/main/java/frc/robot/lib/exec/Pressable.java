package frc.robot.lib.exec;

import java.util.function.BooleanSupplier;

public interface Pressable {

    boolean pressed();
    boolean held();
    boolean released();

    static Pressable from(BooleanSupplier button) {
        return new MyPressable(button);
    }

}
