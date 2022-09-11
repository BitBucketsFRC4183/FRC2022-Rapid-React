package frc.robot.lib.capability.asc;

import java.util.function.Function;

public interface BorrowValue<READ, MODIFY extends READ> {

    READ read(String reason);
    MODIFY readModify(String reason) throws IllegalStateException; //complains



}
