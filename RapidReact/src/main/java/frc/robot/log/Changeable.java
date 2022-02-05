package frc.robot.log;

import edu.wpi.first.networktables.EntryNotification;

import java.util.function.Consumer;

/**
 * Something that can be changed in the dashboard and read from
 *
 * dashboard -> code
 *
 * @param <T> the type of data in the dashbaord
 */
public interface Changeable<T> extends Consumer<EntryNotification> {

    T currentValue();

}
