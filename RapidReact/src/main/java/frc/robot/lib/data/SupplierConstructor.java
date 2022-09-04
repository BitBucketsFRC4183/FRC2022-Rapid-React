package frc.robot.lib.data;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;

import java.util.function.Supplier;

public interface SupplierConstructor<T> {

    Supplier<T> make(ShuffleboardContainer container, String path, T initial);

}
