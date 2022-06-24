package frc.robot.utils;

public enum AutonomousPath
{
    NOTHING("Nothing", "Nothing"),

    HARDCODED("", "Hardcoded"),

    ONE_BALL_PATHPLANNER("1 Ball Auto", "1 Ball - PathPlanner"),
    ONE_BALL_HC("1 Ball Auto", "1 Ball Auto Hardcoded"),
    ONE_BALL_INTAKE("1.5 Ball Auto"),
    TWO_BALL_HANGAR("2 Ball Auto Hangar"),
    TWO_BALL_WALL("2 Ball Auto Wall"),
    THREE_BALL("3 Ball Auto"),
    FOUR_BALL("4 Ball Auto"),
    FOUR_BALL_ALT("4 Ball Auto Alternative"),
    FIVE_BALL("5 Ball Auto");

    public final String pathName;
    public final String dashboardName;

    AutonomousPath(String pathName, String dashboardName)
    {
        this.pathName = pathName;
        this.dashboardName = dashboardName;
    }

    AutonomousPath(String pathName)
    {
        this(pathName, pathName);
    }
}
