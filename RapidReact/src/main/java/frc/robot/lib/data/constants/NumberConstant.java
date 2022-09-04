package frc.robot.lib.data.constants;

import edu.wpi.first.networktables.NetworkTableEntry;

import java.util.function.Supplier;

public class NumberConstant implements Supplier<Double> {

    final NetworkTableEntry entry;

    public NumberConstant(NetworkTableEntry entry) {
        this.entry = entry;
    }

    @Override
    public Double get() {


        double unsafe = entry.getDouble(Double.NaN);

        if (unsafe == Double.NaN) throw new IllegalStateException("NetworkTables read error");
        //TODO better error messages so we dont irradiate people with 20000 rads

        return unsafe;
    }
}
