package frc.robot.system.vision;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.System;

import static frc.robot.system.vision.VisionConstants.*;

public class VisionSubsystem extends SubsystemBase implements System {

    NetworkTableEntry yEntry;

    @Override
    public void init() {
        yEntry = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty");
    }

    @Override
    public void periodic() {

    }

    @Override
    public void stop() {

    }

    public double angleTarget() {
        double ty = yEntry.getDouble(0.0);


// distance from the target to the floor
        double goalHeightInches = 60.0;

        double angleToGoalDegrees = MOUNT_ANGLE_DEGREES + ty;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

//calculate distance
        double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)/Math.tan(angleToGoalRadians);

    }



}
