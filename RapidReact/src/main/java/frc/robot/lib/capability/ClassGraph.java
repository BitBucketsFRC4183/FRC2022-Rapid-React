package frc.robot.lib.capability;

public interface ClassGraph {


    <A, B extends A> void vertex(Class<A> readOnly, Class<B> readWrite);

    void readOnlyEdge(Class<?> resource, Class<?> user);
    void readWriteEdge(Class<?> resource, Class<?> user);
    EdgeBorrow readWriteBorrow(Class<?> resource, Class<?> user);

}
