package frc.robot.subsystem;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.log.LogLevel;
import frc.robot.log.Logger;
import frc.robot.log.SharedLogger;

import java.util.ArrayList;
import java.util.List;
import frc.robot.config.Config;

public abstract class BitBucketsSubsystem extends SubsystemBase
{
    private final Logger logger;
    protected final Config config;

    protected BitBucketsSubsystem(Config config)
    {
        this.setName(this.getClass().getSimpleName());

        this.config = config;

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

}
