package frc.robot.lib.fsm;

import frc.robot.lib.System;

public class TrueStateSystem<T> implements System {

    T state;

    final StateProcessor<T> calculator;
    final StateSystem<T> handler;

    public TrueStateSystem(StateProcessor<T> calculator, StateSystem<T> handler) {
        this.calculator = calculator;
        this.handler = handler;

        state = calculator.defaultState();
    }

    @Override
    public void periodic(float delta, int iteration) {
        T desiredState = calculator.evaluateNextState(state);
        handler.consume(state);
    }
}
