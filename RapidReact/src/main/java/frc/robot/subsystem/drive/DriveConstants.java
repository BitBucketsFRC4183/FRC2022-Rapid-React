package frc.robot.subsystem.drive;

import frc.robot.lib.Constants;
import frc.robot.lib.log.Constant;
import frc.robot.lib.log.DoubleConst;
import frc.robot.lib.log.LogRegister;

import java.util.function.Supplier;

import static com.swervedrivespecialties.swervelib.SdsModuleConfigurations.MK4_L2;

public class DriveConstants implements Constants {

    //you can configure this in smartdashboard, it will also persist between restarts
    static final Constant<Double> TRACK_WIDTH = new DoubleConst(0.6096,"Track Width (m)");
    static final Constant<Double> WHEEL_BASE = new DoubleConst(1,"Wheel Base Size (m)"); //TODO FIX

    static final double MAX_VELOCITY = 6380.0 / 60.0 * MK4_L2.getDriveReduction() * MK4_L2.getWheelDiameter() * Math.PI;
    static final Supplier<Double> MAX_ANG_VEL = () -> MAX_VELOCITY / Math.hypot(TRACK_WIDTH.value() / 2, WHEEL_BASE.value()) / 2;


    @Override
    public void register(LogRegister logRegister) {
        logRegister.log(TRACK_WIDTH, WHEEL_BASE);


    }
}
