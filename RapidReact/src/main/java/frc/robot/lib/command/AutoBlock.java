package frc.robot.lib.command;

import java.util.function.Consumer;

public interface AutoBlock<SCRIPT> {

    AutoBlock<SCRIPT> delay(int seconds);
    AutoBlock<SCRIPT> close();

    AutoBlock<SCRIPT> optional(Consumer<SCRIPT> command);
    AutoBlock<SCRIPT> required(Consumer<SCRIPT> command);
    <X> AutoBlock<X> then(Class<X> clazz);

}
