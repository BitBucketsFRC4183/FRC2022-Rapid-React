package frc.robot.lib.log;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import java.util.function.Supplier;

public interface Logger {

    ShuffleboardTab tab();
    void sendable(Sendable... sendable); //log things that aren't supported


    <T> Supplier<T> constant(Class<T> type, String id, T defaultValue);
    <T> T constantOnce(Class<T> type, String id, T defaultValue);
    <T> Supplier<T> constant(Value<T> value, String id, T defaultValue);

}
