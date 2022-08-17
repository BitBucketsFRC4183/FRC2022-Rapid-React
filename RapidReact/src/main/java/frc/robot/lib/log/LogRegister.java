package frc.robot.lib.log;

import edu.wpi.first.util.sendable.Sendable;

public interface LogRegister {

    void log(Constant<?>... constant);

    void log(Sendable... sendable);

}
