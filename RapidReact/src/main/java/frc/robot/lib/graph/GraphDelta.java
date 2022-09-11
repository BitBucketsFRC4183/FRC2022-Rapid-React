package frc.robot.lib.graph;

public interface GraphDelta {

    void vertexCreate(String name, int id);

    void edgeCreateRead(int parent, int child);
    void edgeCreateModify(int parent, int child);

    void edgeRead(int user, String action);
    void edgeModify(int user, String action);



}
