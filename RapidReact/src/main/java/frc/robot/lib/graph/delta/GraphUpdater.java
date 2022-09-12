package frc.robot.lib.graph.delta;

public interface GraphUpdater {

    void vertexCreate(String name, int vertex);

    void edgeCreateRead(int edge, int parent, int child); //returns an edge
    void edgeCreateModify(int edge, int parent, int child); //returns an edge

    void edgeRead(int edge, String action);
    void edgeModify(int edge, String action);

}
