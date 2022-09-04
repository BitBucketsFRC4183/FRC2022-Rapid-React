package frc.robot.lib.motors;

public interface MotorBuilder<T> {

    //accesses internal motor for config
    T lowLevel();

    //builds config into motor and evicts motor from builder
    T build();
}
