package frc.robot.log;

/**
 * Something that can put data in the dashboard (without reading it afterwards).
 *
 * code -> dashboard
 *
 * @param <T>
 */
public interface Loggable<T> {

    void log(T object);
    void log(LogLevel level, T object);

}
