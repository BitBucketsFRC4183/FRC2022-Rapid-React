package frc.robot.log;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Objects;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

/**
 * inside, is an executor.
 */
public class SharedLogger implements Logger {

    //config
    private static final LogLevel LEVEL = LogLevel.GENERAL;

    private static Executor cachedExecutor; //scalar
    private final String subsystemName;

    public SharedLogger(String subsystemName) {
        this.subsystemName = subsystemName;
    }

    Executor executor() {
        //cringe and bad
        return Objects.requireNonNullElseGet(
                cachedExecutor,
                () -> cachedExecutor = Executors.newSingleThreadExecutor()
        );

    }

    @Override
    public void logString(LogLevel level, String path, String data) {
        if (LEVEL.shouldLog(level)) {
            executor().execute(
                    () -> SmartDashboard.putString(
                            String.format("%s/%s", subsystemName, path), data
                    )
            );
        }
        //does nothing
    }

    @Override
    public void logBool(LogLevel level, String path, boolean data) {
        if (LEVEL.shouldLog(level)) {
            executor().execute(
                    () -> SmartDashboard.putBoolean(
                            String.format("%s/%s", subsystemName, path), data
                    )
            );
        }
    }

    @Override
    public void logNum(LogLevel level, String path, Number data) {
        if (LEVEL.shouldLog(level)) {
            executor().execute(
                    () -> SmartDashboard.putNumber(
                            String.format("%s/%s", subsystemName, path), data.doubleValue()
                    )
            );
        }
    }
}
