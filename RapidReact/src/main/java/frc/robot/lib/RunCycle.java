package frc.robot.lib;

/**
 * Subsystem but better
 */
public interface RunCycle {

    void init();
    void periodic(float delta);
    void stop();

}
