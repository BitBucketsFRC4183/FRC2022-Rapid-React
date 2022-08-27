package frc.robot.subsystem;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.config.Config;

public class VisionSubsystem extends BitBucketsSubsystem {

    NetworkTable limelight;

    public VisionSubsystem(Config config) {
        super(config);
    }

    @Override
    public void init() {
        limelight = NetworkTableInstance.getDefault().getTable("limelight");
    }

    @Override
    public void periodic() {
        System.out.printf("LIMELIGHT TEST: Target x = %s and pipeline %s%n", limelight.getEntry("tx").getDouble(0.0), limelight.getEntry("getpipe").getDouble(0.0));
        System.out.printf("LIMELIGHT TEST: Target y = %s and pipeline %s%n", limelight.getEntry("ty").getDouble(0.0), limelight.getEntry("getpipe").getDouble(0.0));
    }

    @Override
    public void disable() {

    }


}
