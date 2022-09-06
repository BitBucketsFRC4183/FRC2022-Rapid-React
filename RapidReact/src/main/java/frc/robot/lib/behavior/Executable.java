package frc.robot.lib.behavior;

import frc.robot.lib.behavior.CommandIs;
import frc.robot.lib.behavior.CommandShould;
import frc.robot.lib.exec.Execute;

public interface Executable {

    CommandShould run(CommandIs commandIs);

    String describeExecutable();


}
