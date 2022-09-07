package frc.robot.lib.capability;

import frc.robot.lib.RobotInit;
import frc.robot.lib.System;
import frc.robot.lib.header.SystemContext;

public class CapabilityInit implements RobotInit.Setup {

    final SystemContext context;

    public CapabilityInit(SystemContext context) {
        this.context = context;
    }

    //register containers here

    @Override
    public <READ extends System> CapabilityContainer<READ, READ> initReadSystem(SystemMaker<READ> maker) {


        maker.init(context);


        return null;
    }

    @Override
    public <READ extends System, READ_WRITE extends READ> CapabilityContainer<READ, READ_WRITE> initReadWriteSystem(SystemMaker<READ_WRITE> maker) {
        return null;
    }

    @Override
    public void registerOptionalMotorName(int port, String name) {

    }
}
