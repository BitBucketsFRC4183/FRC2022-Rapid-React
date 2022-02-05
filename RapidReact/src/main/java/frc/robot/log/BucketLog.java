package frc.robot.log;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.log.impl.DefaultChangeable;
import frc.robot.log.impl.DefaultLoggable;

import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

public class BucketLog {

    private final static LogLevel baseLogLevel = LogLevel.DEBUG;
    private final static LogLevel defaultLogLevel = LogLevel.DEBUG;

    private final static Executor executor = Executors.newSingleThreadExecutor();

    /**
     * Make a loggable
     * @param put type of data to put
     * @param path the path of the data
     * @param <T> type
     * @return a new loggable
     */
    public static <T> Loggable<T> loggable(Put<T> put, String path) {
        return loggable(defaultLogLevel, put, path);
    }

    /**
     * Make a loggable with default level
     * @param level the default level
     * @param put the data to put
     * @param path the path of the data
     * @param <T> type
     * @return a new loggable
     */
    public static <T> Loggable<T> loggable(LogLevel level, Put<T> put, String path) {
        return new DefaultLoggable<>(executor, path, put, level, baseLogLevel);
    }

    /**
     * Make a changeable, an object representing a value that can be changed in smart dashboard
     * and then read from, expecting an accurate value.
     *
     * @param put type of data to put to dashboard
     * @param path the path of the object
     * @param defaultValue initial data to put
     * @param <T> type
     * @return a changeable
     */
    public static <T> Changeable<T> changeable(Put<T> put, String path, T defaultValue) {

        Changeable<T> changeable = new DefaultChangeable<>(put, defaultValue);
        put.put(path, defaultValue);

        SmartDashboard
                .getEntry(path)
                .addListener(changeable, EntryListenerFlags.kUpdate);

        return changeable;
    }


}
