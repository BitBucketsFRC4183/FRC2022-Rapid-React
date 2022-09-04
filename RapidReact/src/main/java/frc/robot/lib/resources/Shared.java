package frc.robot.lib.resources;

import java.util.function.Consumer;
import java.util.function.Supplier;

public interface Shared<T> extends Supplier<T>, Consumer<T> {
}
