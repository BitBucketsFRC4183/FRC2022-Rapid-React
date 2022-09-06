package frc.robot.lib;

/**
 * Subsystem but better
 */
public interface System {

    default void init() {

    }
    default void periodic(float delta, int iteration) {

    }

    default void stop() {

    }

}
