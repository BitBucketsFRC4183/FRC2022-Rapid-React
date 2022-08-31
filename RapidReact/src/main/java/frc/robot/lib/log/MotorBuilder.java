package frc.robot.lib.log;

public interface MotorBuilder {

    Talon talon(int id);
    Spark spark(int id);



    interface Talon {

    }

    interface Spark {

    }

}
