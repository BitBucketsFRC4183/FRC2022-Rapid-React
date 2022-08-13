package frc.robot.subsystem.drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.RunCycle;

/**
 * Keeps track
 */
public class OdometrySubsystem extends SubsystemBase implements RunCycle {


    AHRS ahrs = new AHRS(I2C.Port.kMXP);


    @Override
    public void init() {

    }

    @Override
    public void periodic(float delta) {

    }

    @Override
    public void stop() {

    }

    public Pose2d absolute() {
        return null; //TODO
    }
}
