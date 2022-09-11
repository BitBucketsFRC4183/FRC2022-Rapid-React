package frc.robot.lib.capability.asc;

//typed
public interface Borrows {


    <READ> BorrowValue<READ, READ> readOnly(Class<READ> read);
    <READ, MODIFY extends READ> BorrowValue<READ, MODIFY> readModify(Class<READ> read, Class<MODIFY> modify);

}
