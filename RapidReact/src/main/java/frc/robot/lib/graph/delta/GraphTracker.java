package frc.robot.lib.graph.delta;

public interface GraphTracker {

    int vertexCreate(String name);

    int edgeCreateRead(int parent, int child); //returns an edge
    int edgeCreateModify(int parent, int child); //returns an edge

    boolean edgeRead(int edge, String action);
    boolean edgeModify(int edge, String action);



}
