package frc.robot.lib.graph.delta;

import frc.robot.lib.System;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicInteger;

public class LockingGraphTracker implements GraphTracker, System {

    final GraphUpdater graphUpdater;

    final List<Integer> modifiable = new ArrayList<>();
    final List<Edge> edges = new ArrayList<>();
    final AtomicInteger vertexCounter = new AtomicInteger(0);
    final AtomicInteger edgeCounter = new AtomicInteger(0);


    public LockingGraphTracker(GraphUpdater graphUpdater) {
        this.graphUpdater = graphUpdater;
    }

    @Override
    public int vertexCreate(String name) {
        int nextId = vertexCounter.incrementAndGet();
        graphUpdater.vertexCreate(name, nextId);

        return nextId;
    }

    int edgeCreate(int parent, int child, boolean writable) {
        Optional<Edge> opt = findEdge(parent, child);

        int returnedEdge;

        if (opt.isEmpty()) {
            Edge edge = new Edge(parent, child, edgeCounter.incrementAndGet(),writable);
            edges.add(edge);
            graphUpdater.edgeCreateRead(edge.edge, edge.parent, edge.child);

            returnedEdge = edge.edge;
        } else {
            returnedEdge = opt.get().edge;
        }

        return returnedEdge;
    }


    @Override
    public int edgeCreateRead(int parent, int child) {
        return edgeCreate(parent, child, false);
    }

    @Override
    public int edgeCreateModify(int parent, int child) {
        return edgeCreate(parent, child, true);
    }

    @Override
    public boolean edgeRead(int user, String action) {
        graphUpdater.edgeRead(user, action);

        return true;
    }

    @Override
    public boolean edgeModify(int user, String action) {
        for (Edge edge : edges) {
           if (edge.edge == user && !edge.modifiable) {
               return false;
           }
        }

        graphUpdater.edgeModify(user, action);

        return true;
    }

    Optional<Edge> findEdge(int parent, int child) {
        for (Edge edge : edges) {
            if (edge.parent == parent && edge.child == child) {
                return Optional.of(edge);
            }
        }

        return Optional.empty();
    }
}
