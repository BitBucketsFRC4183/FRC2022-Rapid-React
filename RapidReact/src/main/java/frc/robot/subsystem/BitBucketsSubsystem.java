package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.log.LogLevel;
import frc.robot.log.Logger;
import frc.robot.log.SharedLogger;

import java.util.ArrayList;
import java.util.List;

public abstract class BitBucketsSubsystem extends SubsystemBase
{
    private final List<BaseTalon> motors;
    private final Logger logger;

    protected BitBucketsSubsystem()
    {
        this.setName(this.getClass().getSimpleName());

        this.motors = new ArrayList<>();
        this.addMotorsToList();


        this.logger = new SharedLogger(getName());
    }

    public Logger logger() {
        return this.logger;
    }

    //When the subsystem is initialized
    public abstract void init();

    //Periodically called
    public abstract void periodic();

    //When the subsystem is turned off
    public abstract void disable();

    //Add motors to list (use this#addMotors() to make it simpler)
    public abstract void addMotorsToList();

    //Helper method that can be used in an implementation of this#addMotorsToList()
    protected void addMotors(BaseTalon... motors)
    {
        this.motors.addAll(List.of(motors));
    }

    //Update all the dashboard constants at once – use this#setDashboardValue() to set each one
    public abstract void updateDashboard();

    //Set a specific dashboard value
    protected <T> void setDashboardValue(String name, T value)
    {
        if(value instanceof Boolean) SmartDashboard.putBoolean(name, (boolean)value);
        else if(value instanceof Integer || value instanceof Double || value instanceof Float) SmartDashboard.putNumber(name, (double)value);
        else if(value instanceof String) SmartDashboard.putString(name, (String)value);
        else if(value instanceof Sendable) SmartDashboard.putData(name, (Sendable)value);
    }

    public List<BaseTalon> getMotors()
    {
        return this.motors;
    }
}
