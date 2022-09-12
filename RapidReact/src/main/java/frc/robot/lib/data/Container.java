package frc.robot.lib.data;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;

import java.util.function.Consumer;
import java.util.function.Supplier;

public interface Container {

    Container GLOBAL = new GlobalContainer();

    Container sub(String path);

    <T> Consumer<T> logger(LoggedConstructor<T> ctor, String path, T emptyValue);
    <IN, OUT> Consumer<OUT> logger(MappedConstructor<IN, OUT> ctor, String path, IN empty);

    <T> Supplier<T> constant(SupplierConstructor<T> ctor, String path, T defaultValue);
    ShuffleboardContainer asShuffleboard();

}
