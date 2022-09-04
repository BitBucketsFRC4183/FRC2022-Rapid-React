package frc.robot.lib.resources;

import java.util.function.Supplier;

public interface SharedOut<T> extends Supplier<T> {

    default T sharedValue() {
        return this.get();
    }

}
