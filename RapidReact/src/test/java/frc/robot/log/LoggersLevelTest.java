package frc.robot.log;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class LoggersLevelTest {

  @Test
  public void logLevelShouldFilterWeakerLevel() {
    assertTrue(LogLevel.GENERAL.shouldLog(LogLevel.CRITICAL)); //logger with filter level general should log a critical msg
    assertTrue(LogLevel.DEBUG.shouldLog(LogLevel.GENERAL)); //logger with debug filter should log a debug msg
    assertTrue(LogLevel.GENERAL.shouldLog(LogLevel.GENERAL)); //logger should log self
    assertFalse(LogLevel.GENERAL.shouldLog(LogLevel.DEBUG)); //base level debug should filter out general
    assertFalse(LogLevel.CRITICAL.shouldLog(LogLevel.GENERAL));
  }
}
