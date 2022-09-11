package frc.robot.lib.capability;

import java.util.List;

public class GraphImpl implements ClassGraph {


    final Class<?> stc;




    @Override
    public <A, B extends A> void vertex(Class<A> readOnly, Class<B> readWrite) {

    }

    @Override
    public void readOnlyEdge(Class<?> resource, Class<?> user) {

    }

    @Override
    public void readWriteEdge(Class<?> resource, Class<?> user) {

    }

    @Override
    public EdgeBorrow readWriteBorrow(Class<?> resource, Class<?> user) {
        return null;
    }
}
