package frc.robot.lib.behavior.impl;

import frc.robot.lib.behavior.CommandIs;
import frc.robot.lib.behavior.CommandShould;
import frc.robot.lib.behavior.Executable;

import java.util.List;

public class SequenceExecutable implements Executable {

    final List<Executable> list;
    int pointer = 0;

    public SequenceExecutable(List<Executable> list) {
        this.list = list;
    }

    @Override
    public CommandShould run(CommandIs commandIs) {

        CommandShould should = list.get(pointer).run(commandIs);

        if (should == CommandShould.COMPLETE) {
            int pointerNext = pointer + 1;
            int lengthNext = pointerNext + 1;

            if (lengthNext > list.size()) {
                pointer = 0;

                return CommandShould.COMPLETE;
            } else {
                pointer = pointerNext;

                return CommandShould.CONTINUE;
            }
        }

        return should;

    }

    @Override
    public String describeExecutable() {
        //TODO append "wrapped" to returned string

        return list.get(pointer).describeExecutable();
    }

}
