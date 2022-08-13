package frc.robot.subsystem.drive;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Buttons;
import frc.robot.lib.SystemHeader;
import frc.robot.lib.log.Constant;
import frc.robot.lib.log.DoubleConst;

import java.util.function.Supplier;

import static com.swervedrivespecialties.swervelib.SdsModuleConfigurations.MK4_L2;

public class DriveHeader implements SystemHeader<DriveSubsystem> {

    //you can configure this in smartdashboard, it will also persist between restarts
    static final Constant<Double> TRACK_WIDTH = new DoubleConst(0.6096,"Track Width (m)");
    static final Constant<Double> WHEEL_BASE = new DoubleConst(0.6096,"Wheel Base Size (m)");

    static final double MAX_VELOCITY = 6380.0 / 60.0 * MK4_L2.getDriveReduction() * MK4_L2.getWheelDiameter() * Math.PI;

    static final Supplier<Double> MAX_ANG_VEL = () -> MAX_VELOCITY / Math.hypot(TRACK_WIDTH.value() / 2, WHEEL_BASE.value()) / 2;


    @Override
    public void init(ShuffleboardTab tab) {

    }

    @Override
    public void buttons(Buttons buttons) {

    }
}
