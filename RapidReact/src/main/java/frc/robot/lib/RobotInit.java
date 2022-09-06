package frc.robot.lib;

import frc.robot.lib.capability.CapabilityContainer;
import frc.robot.lib.header.*;

public interface RobotInit {

    interface Setup {
        <READ extends System> CapabilityContainer<READ, READ> initReadSystem(SystemMaker<READ> maker);
        <READ extends System, READ_WRITE extends READ> CapabilityContainer<READ, READ_WRITE> initReadWriteSystem(SystemMaker<READ_WRITE> maker);

        //THIS IS OPTIONAL motors will still work without this but will just be sad :( and not be logged with a name
        void registerOptionalMotorName(int port, String name); //also allows controlling the motor through debug
    }

    void start(Setup setup);



}
