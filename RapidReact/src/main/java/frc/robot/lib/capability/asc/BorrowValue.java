package frc.robot.lib.capability.asc;

public interface BorrowValue<READ, MODIFY extends READ> {

    READ read(String reason);

    boolean isNotInUse();
    MODIFY readModify(String reason);


}
