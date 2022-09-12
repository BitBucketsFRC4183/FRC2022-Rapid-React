package frc.robot.lib.graph.delta;

import edu.wpi.first.networktables.NetworkTableEntry;

public class MyGraphUpdater implements GraphUpdater {

    final NetworkTableEntry creationUpdater;
    final NetworkTableEntry edgeCreate;

    public MyGraphUpdater(NetworkTableEntry creationUpdater, NetworkTableEntry edgeCreate) {
        this.creationUpdater = creationUpdater;
        this.edgeCreate = edgeCreate;
    }

    @Override
    public void vertexCreate(String name, int vertex) {
        creationUpdater.setStringArray(new String[]{
                name,
                String.valueOf(vertex)
        });
    }

    @Override
    public void edgeCreateRead(int edge, int parent, int child) {
        edgeCreate.setStringArray(new String[]{
                String.valueOf(edge),
                String.valueOf(parent),
                String.valueOf(child),
                String.valueOf(false)
        });
    }

    @Override
    public void edgeCreateModify(int edge, int parent, int child) {
        edgeCreate.setStringArray(new String[]{
                String.valueOf(edge),
                String.valueOf(parent),
                String.valueOf(child),
                String.valueOf(true)
        });
    }

    @Override
    public void edgeRead(int edge, String action) {
        edgeCreate.setStringArray(new String[]{
                String.valueOf(edge),
                action,
                String.valueOf(false)
        });
    }

    @Override
    public void edgeModify(int edge, String action) {
        edgeCreate.setStringArray(new String[]{
                String.valueOf(edge),
                action,
                String.valueOf(true)
        });
    }
}
