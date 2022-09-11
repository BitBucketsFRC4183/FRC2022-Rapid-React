package frc.robot.lib.graph;

public interface Graph {

    <A,B extends A> void addVertex(Class<A> read, Class<B> modifies);
    <A> void addVertex(Class<A> read);

    int addEdgeReadOnly(Class<?> parent, Class<?> child) throws NoSuchVertexException; //edgeID
    int addEdgeModifies(Class<?> parent, Class<?> child) throws NoSuchVertexException; //edgeID

    void usageOfEdge(int which, String usage);


}
