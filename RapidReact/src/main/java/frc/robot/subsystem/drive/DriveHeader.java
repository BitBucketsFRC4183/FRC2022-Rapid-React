package frc.robot.subsystem.drive;

import frc.robot.lib.header.Header;
import frc.robot.lib.System;
import frc.robot.lib.header.MotorBuilder;
import frc.robot.lib.log.Constant;
import frc.robot.lib.log.DoubleConst;
import frc.robot.lib.header.ConstantBuilder;

import java.util.function.Supplier;

import static com.swervedrivespecialties.swervelib.SdsModuleConfigurations.MK4_L2;

public class DriveHeader implements Header {

    //you can configure this in smartdashboard, it will also persist between restarts
    final Constant<Double> TRACK_WIDTH = new DoubleConst(0.6096,"Track Width (m)");
    final Constant<Double> WHEEL_BASE = new DoubleConst(1,"Wheel Base Size (m)"); //TODO FIX

    final double MAX_VELOCITY = 6380.0 / 60.0 * MK4_L2.getDriveReduction() * MK4_L2.getWheelDiameter() * Math.PI;
    final Supplier<Double> MAX_ANG_VEL = () -> MAX_VELOCITY / Math.hypot(TRACK_WIDTH.value() / 2, WHEEL_BASE.value()) / 2;


    @Override
    public System[] init(ConstantBuilder constantBuilder, MotorBuilder motorBuilder) {
        var width = constantBuilder.constant(double.class, "track width", 0.6096);
        constantBuilder.constantOnce(double.class, "eeoo", 2.5);


        return systems(
                new DriveSubsystem(

                )
        );
    }
}
