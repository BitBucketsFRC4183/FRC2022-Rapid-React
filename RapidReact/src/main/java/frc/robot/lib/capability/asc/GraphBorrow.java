package frc.robot.lib.capability.asc;

import org.jgrapht.Graph;

public class GraphBorrow<A,B extends A> implements BorrowValue<A,B> {


    @Override
    public A read(String reason) {
        //update the graph!!!


        return stored;
    }

    @Override
    public B readModify(String reason) throws IllegalStateException {
        return stored;
    }
}
