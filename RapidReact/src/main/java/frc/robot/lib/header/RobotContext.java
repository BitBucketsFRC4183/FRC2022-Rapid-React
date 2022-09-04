package frc.robot.lib.header;

import frc.robot.Buttons;
import frc.robot.lib.values.Values;
import frc.robot.lib.motors.Motors;
import frc.robot.lib.resources.Resources;

public interface RobotContext extends Resources, Values, Motors {

    Buttons buttons();

}
