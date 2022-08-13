package frc.robot.lib;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Buttons;

/**
 * Headers hold static loggers so i don't have to look at them
 * Basically just code organization
 *
 * OFFICIAL DESCRIPTION
 * Headers are ... TODO
 */
public interface SystemHeader<T extends RunCycle> {

    T init(ShuffleboardTab tab);
    void register(T instance, Buttons buttons);

}
