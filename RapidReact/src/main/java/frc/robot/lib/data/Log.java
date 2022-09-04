package frc.robot.lib.data;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import frc.robot.lib.data.log.NumberEntry;
import frc.robot.lib.data.log.TextEntry;

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



    };





}
