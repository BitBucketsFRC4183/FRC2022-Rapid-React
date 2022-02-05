package frc.robot.log.impl;

import frc.robot.log.Buttonable;

import java.util.concurrent.atomic.AtomicBoolean;

public class DefaultButtonable implements Buttonable {

    private final AtomicBoolean atomicBoolean;

    public DefaultButtonable(AtomicBoolean atomicBoolean) {
        this.atomicBoolean = atomicBoolean;
    }

    @Override
    public void toggle() {

        boolean current = atomicBoolean.get();

        while (!atomicBoolean.compareAndSet(current, !current)) {
            current = atomicBoolean.get();
        }
    }

    @Override
    public boolean val() {
        return atomicBoolean.get();
    }
}
