package frc.robot.lib.fsm;

import static org.junit.Assert.*;

public class StateTransitionTest {

    public void shouldTransition() {

        MyStateTransition transition = new MyStateTransition(new FoodSubsystem() {
            @Override
            public boolean foundFood() {
                return true;
            }

            @Override
            public void fillStomach() {

            }

            @Override
            public boolean isStomachFull() {
                return true;
            }
        });

        BobState initialState = BobState.HUNGRY;
        BobState currentState = transition.parse(initialState);

        assertEquals(currentState, BobState.EATING);

        BobState nextState = transition.parse(currentState);

        assertEquals(nextState, BobState.FULL);
    }

}
