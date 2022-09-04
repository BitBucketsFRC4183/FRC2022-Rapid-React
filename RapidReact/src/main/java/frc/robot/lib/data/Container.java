package frc.robot.lib.data;

import java.util.function.Consumer;
import java.util.function.Supplier;

public interface Container {

    Container GLOBAL = new GlobalContainer();

    Container sub(String path);

    <T> Consumer<T> logger(LoggedConstructor<T> ctor, String path, T emptyValue);


    <T> Supplier<T> constant(SupplierConstructor<T> ctor, String path, T defaultValue);

}
