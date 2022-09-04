package frc.robot.lib.resources;

import java.util.function.Consumer;

public interface SharedIn<T> extends Consumer<T> {

    default void set(T t) {
        this.accept(t);
    }

}
