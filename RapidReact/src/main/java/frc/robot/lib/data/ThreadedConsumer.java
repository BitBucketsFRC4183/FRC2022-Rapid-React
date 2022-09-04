package frc.robot.lib.data;

import java.util.concurrent.Executor;
import java.util.function.Consumer;

public class ThreadedConsumer<T> implements Consumer<T> {

    private final Executor executor;
    private final Consumer<T> delegate;

    public ThreadedConsumer(Executor executor, Consumer<T> delegate) {
        this.executor = executor;
        this.delegate = delegate;
    }

    @Override
    public void accept(T t) {
        executor.execute(() -> delegate.accept(t));
    }
}
