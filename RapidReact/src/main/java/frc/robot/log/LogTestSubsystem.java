package frc.robot.log;

import frc.robot.config.Config;
import frc.robot.subsystem.BitBucketsSubsystem;

import java.util.concurrent.atomic.AtomicInteger;

public class LogTestSubsystem extends BitBucketsSubsystem {

  private final AtomicInteger counter = new AtomicInteger();

  private final Loggable<Boolean> logBool = BucketLog.loggable(Put.BOOL, "test/isReady");
  private final Loggable<Double> logNum = BucketLog.loggable(Put.DOUBLE, "test/periodic");


  public LogTestSubsystem(Config config) {
    super(config);
  }

  //simulate cool values for the simulator
  @Override
  public void init() {
    logBool.log(LogLevel.GENERAL, true);
  }

  @Override
  public void periodic() {
    logNum.log(LogLevel.GENERAL, (double) counter.incrementAndGet());
  }

  @Override
  public void disable() {}
}
