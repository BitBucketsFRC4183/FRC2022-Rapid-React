package frc.robot.lib.fsm;

import java.util.List;

public interface StateHandler<T> {

    List<T> thisHandles();

    void onEvent();

}
