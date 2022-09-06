package frc.robot.lib.capability;

public class SingleCapabilityCounter<A,B extends A> implements CapabilityContainer<A,B> {


    final ClassGraph graph;
    final Class<A> baseClazz;
    final B implementation;

    public SingleCapabilityCounter(ClassGraph graph, Class<A> baseClazz, B implementation) {
        this.graph = graph;
        this.baseClazz = baseClazz;
        this.implementation = implementation;
    }

    @Override
    public A read(Class<?> read) {
        graph.readOnlyEdge(baseClazz, read);

        return implementation;
    }

    @Override
    public B readWrite(Class<?> readWrite) {
        graph.readWriteEdge(baseClazz, readWrite);

        return implementation;
    }

    @Override
    public GuardedValue<B> readWriteSafe(Class<?> readWrite) {
        EdgeBorrow borrow = graph.readWriteBorrow(baseClazz, readWrite);

        return new SimpleGuardedValue<>(borrow, implementation);
    }
}
