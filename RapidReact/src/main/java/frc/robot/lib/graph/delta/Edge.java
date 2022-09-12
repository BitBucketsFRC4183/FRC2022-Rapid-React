package frc.robot.lib.graph.delta;

public class Edge {

    final int parent;
    final int child;
    final int edge;
    final boolean modifiable;

    public Edge(int parent, int child, int edge, boolean modifiable) {
        this.parent = parent;
        this.child = child;
        this.edge = edge;
        this.modifiable = modifiable;
    }



}
