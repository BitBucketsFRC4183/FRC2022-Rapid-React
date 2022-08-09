package frc.robot.lib;

import frc.robot.lib.command.AutoBlock;

import java.util.function.Supplier;

public interface Routine {


    void and(Supplier<AutoBlock<Object>> factory);

}
