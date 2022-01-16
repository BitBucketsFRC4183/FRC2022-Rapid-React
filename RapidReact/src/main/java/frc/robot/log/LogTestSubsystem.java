package frc.robot.log;

import frc.robot.config.Config;
import frc.robot.subsystem.BitBucketsSubsystem;

import java.util.concurrent.atomic.AtomicInteger;

public class LogTestSubsystem extends BitBucketsSubsystem {

    final AtomicInteger counter = new AtomicInteger();

    public LogTestSubsystem(Config config) {
        super(config);
    }

    //simulate cool values for the simulator
    @Override
    public void init() {
        logger().logBool(LogLevel.GENERAL, "isReady", true);
    }

    @Override
    public void periodic() {
        logger().logNum(LogLevel.GENERAL, "periodic", counter.incrementAndGet());
    }

    @Override
    public void disable() {

    }

    @Override
    public void updateDashboard() {

    }
}
