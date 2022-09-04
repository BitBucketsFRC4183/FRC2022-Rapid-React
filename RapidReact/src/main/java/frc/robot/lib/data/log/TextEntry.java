package frc.robot.lib.data.log;

import edu.wpi.first.networktables.NetworkTableEntry;

import java.util.function.Consumer;

public record TextEntry(NetworkTableEntry entry) implements Consumer<String> {

    @Override
    public void accept(String s) {
        entry.setString(s);
    }
}
