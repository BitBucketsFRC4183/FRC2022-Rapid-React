package frc.robot.lib.data;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;

import java.util.concurrent.Executor;
import java.util.function.Consumer;
import java.util.function.Supplier;

class CommonContainer implements Container {

    private final ShuffleboardContainer self;
    private final Executor executor;

    public CommonContainer(ShuffleboardContainer self, Executor executor) {
        this.self = self;
        this.executor = executor;
    }

    @Override
    public Container sub(String path) {
        return new CommonContainer(self.getLayout(path), executor);
    }

    @Override
    public <T> Consumer<T> logger(LoggedConstructor<T> ctor, String path, T emptyValue) {
        return new ThreadedConsumer<>(executor, ctor.make(self, path, emptyValue));
    }

    @Override
    public <T> Supplier<T> constant(SupplierConstructor<T> ctor, String path, T defaultValue) {
        return ctor.make(self, path, defaultValue);
    }
}
