package frc.robot.lib;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Buttons;

/**
 * Headers are the public facing class representing LOGIC. Headers may have state, commands, and related
 * constants or loggable values.
 *
 * For each hardware feature a robot has, make a Subsystem. For each goal the robot should achieve, make a Header.
 *
 * Headers should define commands in Executable dsl {@link frc.robot.lib.exec.Executable}
 * These commands can be called at any time by external inputs (buttons or autonomous) so manage state using a
 * {@link frc.robot.lib.fsm.FSM}
 */
public interface SystemHeader<T extends RunCycle> {


    //InitDashboard is called first
    void initDashboard(ShuffleboardTab tab);

    /**
     * Function used to get a reference to a header's contained subsystem for injection into the
     * commandScheduler's runtime. Using this you can guarantee that all constants are ready by the time
     * you want your subsystem to be active
     *
     * @return
     */
    T getSystem();


}
