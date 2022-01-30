package frc.robot.simulator;

import frc.robot.config.Config;
import frc.robot.subsystem.BitBucketsSubsystem;

import java.util.concurrent.atomic.AtomicInteger;

public class SimulatorTestSubsystem extends BitBucketsSubsystem {

  final AtomicInteger counter = new AtomicInteger();

  public SimulatorTestSubsystem(Config config) {
    super(config);
  }

  @Override
  public void init() {}

  @Override
  public void periodic() {
    int num = counter.incrementAndGet();

    //check that if in CI the simulator runs for 15 seconds, pass the simulator test
    if (num > 700) {
      System.exit(0);
    }
  }

  @Override
  public void disable() {}
}
