package frc.robot.simulator;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.config.Config;
import frc.robot.subsystem.BitBucketsSubsystem;

public class SetModeTestSubsystem extends BitBucketsSubsystem {

    public SetModeTestSubsystem(Config config) {
        super(config);
    }

    @Override
    public void init() {
        DriverStation.inAutonomous(true);
    }

    @Override
    public void periodic() {

    }

    @Override
    public void disable() {

    }

}
