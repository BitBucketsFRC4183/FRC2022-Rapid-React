package frc.robot.lib.fsm;

public class MyStateProcessor implements StateProcessor<BobState> {

    private final FoodSubsystem foodSubsystem;

    public MyStateProcessor(FoodSubsystem foodSubsystem) {
        this.foodSubsystem = foodSubsystem;
    }
    //you can technically turn this entire class into just the Parse method, but we're using java
    //so it's better to write clean code and break things down further into methods ;)
    @Override
    public BobState parse(BobState currentState) {
        return switch (currentState) { //interior class overview
            case HUNGRY -> hungry();
            case EATING -> eating();
            case FULL -> full();
        };
    }

    public BobState hungry() {
        if (foodSubsystem.foundFood()) return BobState.EATING; //next state should begin eating
        return BobState.HUNGRY; //bob is still hungry
    }

    public BobState eating() {
        if (!foodSubsystem.isStomachFull()) return BobState.EATING; //i'm still eating
        return BobState.FULL;
    }

    public BobState full() {
        //immediately become hungry again
        return BobState.HUNGRY;
    }




}
