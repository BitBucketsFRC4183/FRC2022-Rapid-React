package frc.robot.log;

public interface Loggable<T> {

    void log(T object);
    void log(LogLevel level, T object);

}
