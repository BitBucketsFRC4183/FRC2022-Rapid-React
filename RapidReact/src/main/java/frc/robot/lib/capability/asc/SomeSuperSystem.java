package frc.robot.lib.capability.asc;

import frc.model.drive.Drive;
import frc.robot.lib.System;

public class SomeSuperSystem implements System {

    final BorrowValue<Drive, Drive.Modify> drive;

    public SomeSuperSystem(BorrowValue<Drive, Drive.Modify> drive) {
        this.drive = drive;
    }

    @Override
    public void periodic(float delta, int iteration) {
        drive.read("hello").currentStates();


        drive.readModify("doStuff").driveAt(states);
    }
}
