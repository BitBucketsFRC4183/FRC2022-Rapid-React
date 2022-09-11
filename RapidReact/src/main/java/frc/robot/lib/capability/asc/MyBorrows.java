package frc.robot.lib.capability.asc;

import org.jgrapht.Graph;

public class MyBorrows implements Borrows {

    final Class<?> borrower; //guarunteed to be vertex
    final Graph<Class<?>, ModifyEdge> graph;

    public MyBorrows(Class<?> borrower, Graph<Class<?>, ModifyEdge> graph) {
        this.borrower = borrower;
        this.graph = graph;
    }

    @Override
    public <READ> BorrowValue<READ, READ> readOnly(Class<READ> read) {
        //make

        ModifyEdge newEdge = new ModifyEdge(true);
        graph.addEdge(borrower, read, newEdge);

        return null;
    }

    @Override
    public <READ, MODIFY extends READ> BorrowValue<READ, MODIFY> readModify(Class<READ> read, Class<MODIFY> modify) {
        return null;
    }
}
