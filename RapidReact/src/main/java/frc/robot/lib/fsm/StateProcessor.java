package frc.robot.lib.fsm;

public interface StateProcessor<STATE> {

    STATE defaultState();

    //called every tick
    STATE evaluateNextState(STATE currentState);

}
