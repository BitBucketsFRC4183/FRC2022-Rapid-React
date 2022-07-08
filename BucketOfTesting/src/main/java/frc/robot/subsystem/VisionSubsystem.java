package frc.robot.subsystem;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

    @Override
    public void periodic() {
        double tx = limelight.getEntry("tx").getDouble(0.0);
        double ty = limelight.getEntry("ty").getDouble(0.0);
        System.out.println(tx + " :x-pos" + "\n" + ty + " :y-pos");
        //System.out.printf("LIMELIGHT TEST: Target area = %s and pipeline %s%n\n", limelight.getEntry("ta").getDouble(0.0), limelight.getEntry("getpipe").getDouble(0.0));
        //TODO store tx / ty /dist to scalar and calculate hood orientation
    }

    public void disable() {

    }


}
