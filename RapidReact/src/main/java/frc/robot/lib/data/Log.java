package frc.robot.lib.data;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.lib.data.log.*;

import java.util.Map;

public interface Log {

    LoggedConstructor<Number> NUMBER = (container, path, def) -> {
        SimpleWidget widget = container.add(path, def);

        return new NumberEntry(widget.getEntry());
    };

    LoggedConstructor<Number> GRAPH_NUMBER = (container, path, def) -> {
        SimpleWidget widget = container.add(path, def);
        widget.withWidget(BuiltInWidgets.kGraph);
        return new NumberEntry(widget.getEntry());
    };

    static LoggedConstructor<Number> DIAL_NUMBER(int min, int max) {
        return (container, path, def) -> {
            SimpleWidget widget = container.add(path, def);
            widget.withWidget(BuiltInWidgets.kDial);
            widget.withProperties(Map.of("Min", min, "Max", max));
            return new NumberEntry(widget.getEntry());
        };
    }

    LoggedConstructor<String> STRING = (c, p, d) -> {
        SimpleWidget widget = c.add(p, d);
        widget.withWidget(BuiltInWidgets.kTextView);
        return new TextEntry(widget.getEntry());
    };

    MappedConstructor<double[], WPI_TalonSRX> TALON = (c,p,d) -> {
        ShuffleboardContainer inner = c.getLayout(p, BuiltInLayouts.kList);

        NetworkTableEntry percentOutput = inner.add("Percent Output", d[PID.PERCENT_OUTPUT])
                .withWidget(BuiltInWidgets.kDial)
                .getEntry();
        NetworkTableEntry voltageOutput = inner.add("Voltage Output", d[PID.VOLTAGE_OUTPUT])
                .withWidget(BuiltInWidgets.kNumberBar)
                .withProperties(Map.of("Min", 0, "Max", 12))
                .getEntry();

        return new TalonEntry(voltageOutput, percentOutput);
    };

    MappedConstructor<double[], WPI_TalonSRX> PID_TALON = (c,p,d) -> {
        ShuffleboardContainer inner = c.getLayout(p, BuiltInLayouts.kList);

        NetworkTableEntry percentOutput = inner.add("Percent Output", d[PID.PERCENT_OUTPUT])
                .withWidget(BuiltInWidgets.kDial)
                .getEntry();
        NetworkTableEntry voltageOutput = inner.add("Voltage Output", d[PID.VOLTAGE_OUTPUT])
                .withWidget(BuiltInWidgets.kNumberBar)
                .withProperties(Map.of("Min", 0, "Max", 12))
                .getEntry();

        NetworkTableEntry closeLoop = inner.add("Closed Loop (Target, Error)", d[PID.CLOSED_LOOP])
                .withWidget(BuiltInWidgets.kGraph)
                .getEntry();


        NetworkTableEntry velocity = inner.add("Velocity (Target, Error)", d[PID.VELOCITY]).getEntry();
        NetworkTableEntry position = inner.add("Position (Target, Error)", d[PID.POSITION]).getEntry();

        return new TalonClosedLoopEntry(voltageOutput, percentOutput, closeLoop, velocity, position);
    };





}
