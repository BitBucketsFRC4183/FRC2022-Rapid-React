package frc.robot.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.lib.System;

public class VisionSystem implements System {

    public double getHeadingOffsetDeg() {

        Rotation2d rotation2d = new Rotation2d();

        return 0;
    }

    public boolean hasTarget() {
        return false;
    }

}
