package frc.robot.lib.header;

public interface MotorBuilder {

    Talon talon(int id);
    Spark spark(int id);



    interface Talon {

    }

    interface Spark {

    }

}
