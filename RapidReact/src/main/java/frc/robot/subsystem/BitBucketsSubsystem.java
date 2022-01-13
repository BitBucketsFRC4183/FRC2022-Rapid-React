package frc.robot.subsystem;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class BitBucketsSubsystem extends SubsystemBase
{
    protected BitBucketsSubsystem()
    {
        this.setName(this.getClass().getSimpleName());
    }

    //When the subsystem is initialized
    public abstract void init();

    //Periodically called
    public abstract void periodic();

    //When the subsystem is turned off
    public abstract void disable();

    //Update all the dashboard constants at once – use this#setDashboardValue() to set each one
    public abstract void updateDashboard();

    //Set a specific dashboard value
    protected <T> void setDashboardValue(String name, T value)
    {
        if(value instanceof Boolean bool) SmartDashboard.putBoolean(name, bool);
        else if(value instanceof Integer num) SmartDashboard.putNumber(name, num);
        else if(value instanceof Double num) SmartDashboard.putNumber(name, num);
        else if(value instanceof Float num) SmartDashboard.putNumber(name, num);
        else if(value instanceof String str) SmartDashboard.putString(name, str);
        else if(value instanceof Sendable s) SmartDashboard.putData(name, s);
    }
}
