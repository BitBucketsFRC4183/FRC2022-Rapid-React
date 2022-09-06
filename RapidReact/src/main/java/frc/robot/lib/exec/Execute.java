package frc.robot.lib.exec;

import frc.robot.lib.behavior.Executable;

public interface Execute {

    <T extends Executable> void startCommand(Class<T> type);

}
