package frc.robot.subsystem;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import frc.robot.config.Config;

public class AutonomousSubsystem extends BitBucketsSubsystem {

  public AutonomousSubsystem(Config config) {
    super(config);
  }

  public PathPlannerTrajectory buildPath(String pathName)
  {
    return pathName.equals(this.config.auto.nothingPath)
            ? PathPlanner.loadPath(pathName, 0, 0)
            : PathPlanner.loadPath(pathName, this.config.auto.maxPathFollowVelocity, this.config.auto.maxPathFollowAcceleration);
  }

  @Override
  public void init() {}

  @Override
  public void periodic() {}

  @Override
  public void disable() {}
}
