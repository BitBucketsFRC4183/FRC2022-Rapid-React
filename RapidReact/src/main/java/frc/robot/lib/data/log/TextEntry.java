package frc.robot.lib.data.log;

import edu.wpi.first.networktables.NetworkTableEntry;

import java.util.function.Consumer;

public class TextEntry implements Consumer<String> {

    final NetworkTableEntry entry;

    public TextEntry(NetworkTableEntry entry) {
        this.entry = entry;
    }

    @Override
    public void accept(String s) {
        entry.setString(s);
    }
}
