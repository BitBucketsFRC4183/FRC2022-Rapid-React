package frc.robot.lib.capability.asc;

import org.jgrapht.graph.DefaultEdge;

import java.io.Serializable;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;

public class ModifyEdge extends DefaultEdge implements Serializable {

    final boolean readOnly;

    public ModifyEdge(boolean readOnly) {
        this.readOnly = readOnly;
    }

    //TODO

    Queue<String> reads = new ArrayDeque<>();

    void add(String str) {
        reads.add(str);
    }


}
