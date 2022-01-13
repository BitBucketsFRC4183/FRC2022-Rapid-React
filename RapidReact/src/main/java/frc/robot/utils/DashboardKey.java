package frc.robot.utils;

public enum DashboardKey
{
    FEEDER_STATE("Feeder State"),
    SHOOTER_STATE("Shooter State");

    public String name;

    DashboardKey(String name)
    {
        this.name = name;
    }
}
