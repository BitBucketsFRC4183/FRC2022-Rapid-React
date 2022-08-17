package frc.robot.lib.fsm;

public interface StateTransition<STATE> {

    STATE parse(STATE current);

}
