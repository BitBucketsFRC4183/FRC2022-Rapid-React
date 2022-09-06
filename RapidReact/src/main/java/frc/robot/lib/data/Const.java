package frc.robot.lib.data;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import frc.robot.lib.data.constants.NumberConstant;

import java.util.Map;

public interface Const {

    SupplierConstructor<Double> NUMBER = (c, p, d) -> {
        NetworkTableEntry e = c.addPersistent(p, d).getEntry();

        return new NumberConstant(e);
    };

    SupplierConstructor<Double> PERCENT_NUMBER = (c,p,d) -> {
        NetworkTableEntry e = c
                .addPersistent(p, d)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0.0))
                .getEntry();

        return new NumberConstant(e);
    };

}
