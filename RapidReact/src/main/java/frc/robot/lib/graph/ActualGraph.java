package frc.robot.lib.graph;

import java.util.*;
import java.util.concurrent.atomic.AtomicInteger;

public class ActualGraph implements Graph{

    //local storage
    final Map<Class<?>, String> classToVertexMap = new HashMap<>();
    @Override
    public <A, B extends A> void addVertex(Class<A> read, Class<B> modifies) {
        String name = read.getSimpleName();

        classToVertexMap.put(read, name);
        classToVertexMap.put(modifies, name); //identify the same

        //TODO network queue
        //queue.submit(name)

        vertexDelta.add(name); //here is a new vertex!
    }

    @Override
    public <A> void addVertex(Class<A> read) {
        String name = read.getSimpleName();

        classToVertexMap.put(read, name);

        //TODO network queue

        vertexDelta.add(name);
    }

    @Override
    public int addEdgeReadOnly(Class<?> parent, Class<?> child) throws NoSuchVertexException {
        String parentVertex = classToVertexMap.get(parent);
        String childVertex = classToVertexMap.get(child);

        if (parentVertex == null) throw new NoSuchVertexException("Parent vertex does not exist!");
        if (childVertex == null) throw new NoSuchVertexException("Child vertex does not exist!");

        //edge check
        //edge delta

        return 0;
    }

    @Override
    public int addEdgeModifies(Class<?> parent, Class<?> child) {
        return 0;
    }

    @Override
    public void usageOfEdge(int which, String usage) {

    }
}
