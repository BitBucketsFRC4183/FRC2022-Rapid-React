package frc.model.drive;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface Drive {

    //anyone can read this
    SwerveModuleState[] currentStates();

    //these functions modify the drive
    interface Modify extends Drive {

        void driveAt(SwerveModuleState[] states);

    }

}
