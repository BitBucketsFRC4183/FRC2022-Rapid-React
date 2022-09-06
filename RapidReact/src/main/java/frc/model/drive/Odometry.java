package frc.model.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface Odometry {

    Rotation2d rotation();

    //TODO should be in it's own system
    SwerveModuleState[] calculateFieldOrientedDesiredStates(double x, double y, double rot, double gain);

}
