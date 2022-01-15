package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.Config;

import java.util.ArrayList;
import java.util.List;

public abstract class BitBucketsSubsystem extends SubsystemBase
{
    protected final Config config;
    private final List<BaseTalon> motors;

    protected BitBucketsSubsystem(Config config)
    {
        this.setName(this.getClass().getSimpleName());

        this.config = config;

        this.motors = new ArrayList<>();
        this.addMotorsToList();
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
