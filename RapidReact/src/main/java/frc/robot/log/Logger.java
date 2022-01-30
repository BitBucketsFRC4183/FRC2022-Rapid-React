package frc.robot.log;

import edu.wpi.first.util.sendable.Sendable;

import java.util.function.Consumer;

public interface Logger {

  void subscribeNum(String path, Consumer<Double> consumer, Double defaultDouble);
  void subscribeString(String path, Consumer<String> consumer, String defaultString);
  void subscribeBool(String path, Consumer<Boolean> consumer, Boolean defaultBool);
}
