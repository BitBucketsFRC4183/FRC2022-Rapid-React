package frc.robot.lib.data;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import java.util.function.Consumer;
import java.util.function.Supplier;

class GlobalContainer implements Container {

    private final Executor thread = Executors.newSingleThreadExecutor();

    @Override
    public Container sub(String path) {
        return new CommonContainer(Shuffleboard.getTab(path), thread);
    }

    @Override
    public <T> Consumer<T> logger(LoggedConstructor<T> ctor, String path, T emptyValue) {
        return new ThreadedConsumer<>(thread, ctor.make(Shuffleboard.getTab("global"), path, emptyValue));
    }

    @Override
    public <IN, OUT> Consumer<OUT> logger(MappedConstructor<IN, OUT> ctor, String path, IN empty) {
        return new ThreadedConsumer<>(thread, ctor.make(Shuffleboard.getTab("global"), path, empty));
    }

    @Override
    public <T> Supplier<T> constant(SupplierConstructor<T> ctor, String path, T defaultValue) {
        return ctor.make(Shuffleboard.getTab("global"), path, defaultValue);
    }
}
