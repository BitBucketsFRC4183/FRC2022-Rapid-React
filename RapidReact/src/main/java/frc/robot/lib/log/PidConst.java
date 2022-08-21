package frc.robot.lib.log;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import static frc.robot.utils.ArrayConstants.*;

public class PidConst implements Constant<double[]> {

    private final String key;
    private final double[] defaultValues;

    public PidConst(String key, double[] defaultValues) {
        this.key = key;
        this.defaultValues = defaultValues;
    }

    private NetworkTableEntry p;
    private NetworkTableEntry i;
    private NetworkTableEntry d;

    @Override
    public double[] value() {
        if (p == null) return defaultValues; //TODO make it better

        return new double[] {
                p.getDouble(defaultValues[P]),
                i.getDouble(defaultValues[I]),
                d.getDouble(defaultValues[D])
        };
    }

    @Override
    public void init(ShuffleboardContainer container) {
        ShuffleboardContainer pidBoard = container.getLayout(key);

        p = pidBoard.addPersistent("p", defaultValues[P]).getEntry();
        i = pidBoard.addPersistent("i", defaultValues[I]).getEntry();
        d = pidBoard.addPersistent("d", defaultValues[D]).getEntry();
    }
}
