package frc.robot.log.impl;

import edu.wpi.first.networktables.EntryNotification;
import frc.robot.log.Changeable;
import frc.robot.log.Put;

public class DefaultChangeable<T> implements Changeable<T> {

    private final Put<T> put;

    private volatile T reference;

    public DefaultChangeable(Put<T> put, T reference) {
        this.put = put;
        this.reference = reference;
    }

    @Override
    public T currentValue() {
        return reference;
    }

    @Override
    public void accept(EntryNotification entryNotification) {
        Object object = entryNotification.value.getValue();

        reference = put.convert(object);
    }
}
