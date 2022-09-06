package frc.robot.lib.capability;

public interface GuardedValue<T> {

    boolean tryAcquire(String reason);

    T acquire();

}
