package frc.robot.lib.resources;

import java.util.function.Supplier;

//only use this if you absolutely have to, otherwise "shared data" between systems likely actually belongs in
//the overarching command hub that stores it
public interface Resources {

    @Deprecated
    <T> Shared<T> uncheckedShared(String key); //share a value without checking, used to get around the borrow checker ;)
    //oh rust my love

    <T> void shareInput(String key, Supplier<T> supplier); //share input as a function. Prefer this.
    <T> SharedOut<T> shareOutput(String key); //and multiple outputs
    //if you have more than 1 input an error is thrown
    //if you have an output but no input an error is thrown

    //SHARED FUNCTIONS SHOULD BE GETTERS




}
