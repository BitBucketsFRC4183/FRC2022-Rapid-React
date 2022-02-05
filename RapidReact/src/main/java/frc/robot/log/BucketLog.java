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

    public static <T> Loggable<T> loggable(Put<T> put, String path) {
        return new DefaultLoggable<>(executor, path, put, defaultLogLevel, baseLogLevel);
    }

    public static <T> Changeable<T> changeable(Put<T> put, String path, T defaultValue) {

        Changeable<T> changeable = new DefaultChangeable<>(put, defaultValue);
        put.put(path, defaultValue);

        SmartDashboard
                .getEntry(path)
                .addListener(changeable, EntryListenerFlags.kUpdate);

        return changeable;
    }

}
