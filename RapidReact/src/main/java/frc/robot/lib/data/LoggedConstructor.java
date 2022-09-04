package frc.robot.lib.data;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;

import java.util.function.Consumer;

public interface LoggedConstructor<T> {

    Consumer<T> make(ShuffleboardContainer container, String path, T def);

}
