package frc.robot.utils;

public enum AutonomousPath
{
    NOTHING("Nothing", "Nothing"),
    ONE_BALL__PRELOAD_TARMAC("1 Preload and Tarmac", "One Ball Auto: Exit Tarmac"),
    ONE_BALL__PRELOAD_TARMAC_INTAKE("1.5 Preload, Tarmac, Intake Only", "One Ball Auto: Exit Tarmac, Intake"),
    TWO_BALL__PRELOAD_TARMAC_INTAKE_SHOOT("2 Preload, Tarmac, Intake, Shoot", "Two Ball Auto: Exit Tarmac, Intake, Shoot Ball");

    public final String pathName;
    public final String dashboardName;

    AutonomousPath(String pathName, String dashboardName)
    {
        this.pathName = pathName;
        this.dashboardName = dashboardName;
    }
}
