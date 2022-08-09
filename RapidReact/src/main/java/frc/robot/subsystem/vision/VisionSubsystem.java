package frc.robot.subsystem.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.config.Config;
import frc.robot.subsystem.BitBucketsSubsystem;

public class VisionSubsystem extends BitBucketsSubsystem {

    NetworkTable limelight;

    private final float angleDegrees;
    private final float heightInches;

    public VisionSubsystem(Config config, float angleDegrees, float heightInches) {
        super(config);
        this.angleDegrees = angleDegrees;
        this.heightInches = heightInches;
    }



    @Override
    public void init() {
        limelight = NetworkTableInstance.getDefault().getTable("limelight");
    }

    @Override
    public void periodic() {
        System.out.printf("LIMELIGHT TEST: Target area = %s and pipeline %s%n", limelight.getEntry("ta"), limelight.getEntry("getpipe"));
    }

    @Override
    public void disable() {

    }

    public double angleTarget() {

    }

    public float distanceFromTarget() {
        limelight.getEntry("ty");

        return 0;
    }


}
