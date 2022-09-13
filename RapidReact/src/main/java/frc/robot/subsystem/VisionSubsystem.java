package frc.robot.subsystem;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.config.Config;

public class VisionSubsystem extends BitBucketsSubsystem {

    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private NetworkTableEntry hasTarget;

    public VisionSubsystem(Config config) {
        super(config);
    }

    public boolean hasTarget(){
        return hasTarget.getDouble(0) == 1;
    }

    public double distance() {
        double angleToGoalRad = Math.toRadians( ty.getDouble(0) + 29.8 ) ; //add limelight angle to constant
        double heightGoalTrigMeters = 2.7178 - 0.9398; //goal height - mounting height

        //simple trig TODO optimize and normalize
        return heightGoalTrigMeters / Math.tan(angleToGoalRad);
    }

    public double tx() {
        return tx.getDouble(0);
    }

    @Override
    public void init() {
        NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
        this.tx = limelight.getEntry("tx");
        this.ty = limelight.getEntry("ty");
        this.hasTarget = limelight.getEntry("tv");
    }

    @Override
    public void periodic() {

    }

    @Override
    public void disable() {

    }
}
