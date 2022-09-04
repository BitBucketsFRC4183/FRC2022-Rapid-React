package frc.robot.lib.values;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import java.util.function.Supplier;

public interface Values {

    ShuffleboardTab tab();

    <T> Supplier<T> constant(Class<T> type, String id, T defaultValue);
    <T> T constantOnce(Class<T> type, String id, T defaultValue);
    <T> Supplier<T> constant(Value<T> value, String id, T defaultValue);

}
