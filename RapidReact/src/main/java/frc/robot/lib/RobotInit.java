package frc.robot.lib;

import frc.robot.lib.header.*;

public interface RobotInit {

    interface Setup {
        void registerSubsystem(SystemHeader header);
        void registerDirty(DirtyHeader dirtyHeader); //register a "dirty" subsystem for quick testing and rapid debug


        //commands!!!!!
        <X,A> X registerTeleop(Class<A> system1, CommandsHeader1<X,A> header); //register commands!
        <X,A,B> X registerTeleop(Class<A> system1, Class<B> system2, CommandsHeader2<X,A,B> header);
        <X,A,B,C> X registerTeleop(Class<A> system1, Class<B> system2, Class<C> system3, CommandsHeader3<X,A,B,C> header);

        void registerAuto(); //TODO


        //THIS IS OPTIONAL motors will still work without this but will just be sad :( and not be logged with a name
        void registerOptionalMotorName(int port, String name); //also allows controlling the motor through debug
    }

    void start(Setup setup);



}
