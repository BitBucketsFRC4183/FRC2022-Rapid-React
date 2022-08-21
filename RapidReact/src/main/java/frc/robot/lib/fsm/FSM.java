package frc.robot.lib.fsm;

public interface FSM<STATE> {

    void changeTo(STATE state);
    STATE current();

}
