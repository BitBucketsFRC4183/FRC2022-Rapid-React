package frc.robot.lib.capability;

public interface CapabilityContainer<READ, READ_WRITE extends READ> {


    READ_WRITE readWrite(Class<?> readWrite);
    GuardedValue<READ_WRITE> readWriteSafe(Class<?> readWrite);

}
