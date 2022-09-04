package frc.robot.lib.data.log;

import edu.wpi.first.networktables.NetworkTableEntry;

import java.util.function.Consumer;

public record NumberEntry(NetworkTableEntry singularNumber) implements Consumer<Number> {

    @Override
    public void accept(Number number) {
        singularNumber.setNumber(number);
    }

}
