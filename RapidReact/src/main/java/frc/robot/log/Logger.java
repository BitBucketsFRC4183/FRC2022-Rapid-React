package frc.robot.log;

public interface Logger {

    /**
     * Log a string
     * @param path the path, such as "motorSpeed"
     * @param data the data, such as "500rpm"
     */
    void logString(LogLevel level, String path, String data);
    void logBool(LogLevel level, String path, boolean data);
    void logNum(LogLevel level, String path, Number data);

}
