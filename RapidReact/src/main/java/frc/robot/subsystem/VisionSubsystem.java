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
        System.out.printf("LIMELIGHT TEST: Target area = %s and pipeline %s%n", limelight.getEntry("ta"), limelight.getEntry("getpipe"));
        //TODO store tx / ty /dist to scalar and calculate hood orientation
    }

    @Override
    public void disable() {

    }


}
