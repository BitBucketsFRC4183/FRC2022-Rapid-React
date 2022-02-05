package frc.robot.log;

import edu.wpi.first.networktables.EntryNotification;

import java.util.function.Consumer;

public interface Changeable<T> extends Consumer<EntryNotification> {

    T currentValue();

}
