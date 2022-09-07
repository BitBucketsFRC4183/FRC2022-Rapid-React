package frc.robot.lib.data.log;

import edu.wpi.first.networktables.NetworkTableEntry;

import java.util.function.Consumer;

public class NumberEntry implements Consumer<Number> {

    final NetworkTableEntry singularNumber;

    public NumberEntry(NetworkTableEntry singularNumber) {
        this.singularNumber = singularNumber;
    }

    @Override
    public void accept(Number number) {
        singularNumber.setNumber(number);
    }

}
