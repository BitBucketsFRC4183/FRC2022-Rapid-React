package frc.robot.log;

import edu.wpi.first.util.sendable.Sendable;


import edu.wpi.first.util.sendable.Sendable;

import java.util.function.Consumer;

public interface Logger {

    /**
     * Log a string
     * @param path the path, such as "motorSpeed"
     * @param data the data, such as "500rpm"
     */
    void logString(LogLevel level, String path, String data);
    void logBool(LogLevel level, String path, boolean data);
    void logNum(LogLevel level, String path, Number data);
    void logSend(LogLevel level, String path, Sendable sendable);

    void subscribeNum(String path, Consumer<Double> consumer, Double defaultDouble);
    void subscribeString(String path, Consumer<String> consumer, String defaultString);
    void subscribeBool(String path, Consumer<Boolean> consumer, Boolean defaultBool);
}
