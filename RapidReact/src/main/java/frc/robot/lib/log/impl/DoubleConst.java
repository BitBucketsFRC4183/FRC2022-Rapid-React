package frc.robot.lib.log.impl;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;

public class DoubleConst implements Constant<Double> {

    private final double defaultVal;
    private final String id;

    public DoubleConst(double defaultVal, String id) {
        this.defaultVal = defaultVal;
        this.id = id;
    }

    private NetworkTableEntry entry;

    @Override
    public void init(ShuffleboardContainer container) {
        entry = container.addPersistent(id, defaultVal).getEntry();
    }

    @Override
    public Double value() {

        if (entry == null) return defaultVal;

        return entry.getDouble(defaultVal);
    }
}
