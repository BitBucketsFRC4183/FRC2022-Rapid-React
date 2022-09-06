package frc.robot.lib.fsm;

public interface StateSystem<STATE> {

    void consume(STATE state);

}
