package frc.robot.lib.header;

import frc.robot.lib.capability.asc.BorrowValue;

public interface SystemContext {

    <READ> BorrowValue<READ, READ> borrowReadOnly();
    <READ, MODIFY extends READ> BorrowValue<READ, MODIFY> borrowReadWrite();

}
