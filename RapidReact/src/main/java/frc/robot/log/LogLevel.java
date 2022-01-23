package frc.robot.log;

public enum LogLevel {
  DEBUG(0),
  GENERAL(1),
  CRITICAL(2);

  private final int level;

  LogLevel(int level) {
    this.level = level;
  }

  boolean shouldLog(LogLevel logLevel) {
    return logLevel.level >= this.level;
  }
}
