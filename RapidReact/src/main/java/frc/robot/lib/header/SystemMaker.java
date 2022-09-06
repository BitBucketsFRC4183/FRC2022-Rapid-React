package frc.robot.lib.header;

/**
 * header
 */
public interface SystemMaker<T> {

    Class<T> modelType();
    T init(SystemContext context);

}
