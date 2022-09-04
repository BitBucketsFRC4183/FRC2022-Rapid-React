package frc.robot.lib.data;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;

import java.util.function.Consumer;

public interface MappedConstructor<IN, OUT> {

    Consumer<OUT> make(ShuffleboardContainer container, String path, IN def);

}
