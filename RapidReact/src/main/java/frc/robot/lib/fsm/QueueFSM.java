package frc.robot.lib.fsm;

import frc.robot.lib.System;

import java.util.LinkedList;
import java.util.Queue;

public class QueueFSM<T> implements FSM<T>, System {

    private final StateTransition<T> transition;
    private final Queue<T> commandQueue;

    private T currentState;

    public QueueFSM(StateTransition<T> transition, T defaultState) {
        this.transition = transition;
        this.currentState = defaultState;
        this.commandQueue = new LinkedList<>();
    }

    @Override
    public void changeTo(T t) {
        commandQueue.add(t);
    }

    @Override
    public T current() {
        return currentState;
    }

    @Override
    public void periodic(float delta) {
        T t;
        if ((t = commandQueue.poll()) != null) {
            //use queue
            currentState = transition.parse(t);
            return;

        }

        currentState = transition.parse(currentState);
    }
}
