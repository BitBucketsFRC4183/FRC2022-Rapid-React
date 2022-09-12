package frc.model.drive;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface Drive {

    SwerveModuleState[] currentStates();

    interface Modify extends Drive {

        void driveAt(SwerveModuleState[] states);

    }

}
