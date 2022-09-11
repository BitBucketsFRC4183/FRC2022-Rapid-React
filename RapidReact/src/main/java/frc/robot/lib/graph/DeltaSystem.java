package frc.robot.lib.graph;

import edu.wpi.first.networktables.*;

public class DeltaSystem implements GraphDelta {

    final NetworkTableEntry vertexDelta; //StringArray
    final NetworkTableEntry edgeDelta;

    public DeltaSystem(NetworkTableEntry vertexDelta, NetworkTableEntry edgeDelta) {
        this.vertexDelta = vertexDelta;
        this.edgeDelta = edgeDelta;
    }

    @Override
    public void vertexCreate(String name, int id) {
        String[] update = new String[] {name, id + ""}; //message

        vertexDelta.setStringArray(update);
    }

    @Override
    public void edgeCreateRead(int parent, int child) {
        Integer[] edge = new Integer[] {parent, child, 0}; //0 is "not writing"

        edgeDelta.setNumberArray(edge);
    }

    @Override
    public void edgeCreateModify(int parent, int child) {
        Integer[] edge = new Integer[] {parent, child, 1}; //0 is "writing"

        edgeDelta.setNumberArray(edge);
    }
}
