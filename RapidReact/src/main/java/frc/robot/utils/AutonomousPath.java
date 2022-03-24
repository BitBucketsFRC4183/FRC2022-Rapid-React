package frc.robot.utils;

public enum AutonomousPath
{
    NOTHING("Nothing", "Nothing"),

    TEST_1M_FORWARD("Test Path (1m Forward)", "Test Path: 1m Forward"),
    TEST_1M_FORWARD_1M_UP("Test Path (1m Forward, 1m Up)", "Test Path: 1m Forward, 1m Up"),

    HARDCODED("", "Hardcoded"),

    ONE_BALL("1 Ball Auto"),
    ONE_BALL_INTAKE("1.5 Ball Auto"),
    TWO_BALL_HANGAR("2 Ball Auto Hangar"),
    TWO_BALL_WALL("2 Ball Auto Wall"),
    THREE_BALL("3 Ball Auto"),
    FOUR_BALL("4 Ball Auto"),
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
