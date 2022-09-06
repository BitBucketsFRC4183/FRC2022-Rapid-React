package frc.robot.lib.capability;

import java.util.function.BooleanSupplier;

public interface EdgeBorrow {

    boolean tryBorrow(String reason);
    boolean safe();

}
