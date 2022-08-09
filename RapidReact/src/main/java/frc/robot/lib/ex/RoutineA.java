package frc.robot.lib.ex;

import frc.robot.lib.Routine;
import frc.robot.lib.command.AutoBlock;

import java.util.function.Supplier;

public class RoutineA implements Routine {
    @Override
    public void and(Supplier<AutoBlock<Object>> factory) {
        factory.get()
                .then(MonadScriptA.class)
                .optional(MonadScriptA::turnOrange)
                .delay(10)
                .optional(MonadScriptA::turnRed);
    }
}
