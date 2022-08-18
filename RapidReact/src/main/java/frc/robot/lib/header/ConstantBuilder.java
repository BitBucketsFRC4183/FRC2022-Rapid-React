package frc.robot.lib.header;

import edu.wpi.first.util.sendable.Sendable;
import frc.robot.lib.log.Constant;

public interface ConstantBuilder {

    //sendable support
    void logOnce(Sendable... sendable); //log things that aren't supported

    /**
     * Gets a persistant constant from Shuffleboard that can change during periodic
     * @param type the type of data used
     * @param id the name SmartDashboard should assign
     * @param defaultValue the default value if no other value is present
     * @return what to return
     * @param <T> type
     */
    <T> Constant<T> constant(Class<T> type, String id, T defaultValue);

    //Gets a constant as value once from dashboard that cannot change during periodic
    <T> T constantOnce(Class<T> type, String id, T defaultValue);

}
