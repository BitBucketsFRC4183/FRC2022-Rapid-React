package frc.robot.lib.data;

import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.lib.data.constants.NumberConstant;

public interface Const {

    SupplierConstructor<Double> NUMBER = (c, p, d) -> {
        NetworkTableEntry e = c.addPersistent(p, d).getEntry();

        return new NumberConstant(e, d);
    };

}
