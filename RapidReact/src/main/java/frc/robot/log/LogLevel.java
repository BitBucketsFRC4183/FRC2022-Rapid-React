package frc.robot.log;

/**
 * Level to log at
 */
public enum LogLevel {
  DEBUG(0),
  GENERAL(1),
  CRITICAL(2);

  private final int level;

  LogLevel(int level) {
    this.level = level;
  }

  public boolean shouldLog(LogLevel logLevel) {
    return logLevel.level >= this.level;
  }
}
