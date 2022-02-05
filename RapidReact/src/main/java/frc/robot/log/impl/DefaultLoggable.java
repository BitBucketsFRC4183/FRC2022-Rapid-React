package frc.robot.log.impl;

import frc.robot.log.LogLevel;
import frc.robot.log.Loggable;
import frc.robot.log.Put;

import java.util.concurrent.Executor;

public class DefaultLoggable<T> implements Loggable<T> {

    private final Executor executor;

    private final String path;
    private final Put<T> put;
    private final LogLevel defaultLevel;
    private final LogLevel baseLevel;

    public DefaultLoggable(Executor executor, String path, Put<T> put, LogLevel defaultLevel, LogLevel baseLevel) {
        this.executor = executor;
        this.path = path;
        this.put = put;
        this.defaultLevel = defaultLevel;
        this.baseLevel = baseLevel;
    }

    @Override
    public void log(T object) {
        log(defaultLevel, object);
    }

    @Override
    public void log(LogLevel level, T object) {
        if (baseLevel.shouldLog(level)) {
            executor.execute(() -> {
                put.put(path, object);
            });
        }
    }
}
