package frc.robot.lib.header;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.lib.log.Constant;
import frc.robot.lib.log.Yield;

public interface LogBuilder {

    ShuffleboardTab tab();
    void logOnce(Sendable... sendable); //log things that aren't supported

    <T> Constant<T> constant(Class<T> type, String id, T defaultValue);
    <T> T constantOnce(Class<T> type, String id, T defaultValue);
    <T> Constant<T> constant(Yield<T> yield, String id, T defaultValue);

}
