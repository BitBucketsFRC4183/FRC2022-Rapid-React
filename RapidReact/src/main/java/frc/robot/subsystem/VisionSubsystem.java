package frc.robot.subsystem;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.Config;

public class VisionSubsystem extends BitBucketsSubsystem {

  private NetworkTable limelightTable;
  private NetworkTableEntry tx;
  private NetworkTableEntry ty;
  private NetworkTableEntry ta;
  private NetworkTableEntry tv;

  private double horizontalTargetAngle = 0.0;
  private double verticalTargetAngle = 0.0;
  private double targetArea = 0.0;
  private boolean targetValid = false;

  private double targetDistance = 0;

  protected VisionSubsystem(Config config) {
    super(config);
  }

  @Override
  public void init() {
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    tx = limelightTable.getEntry("tx");
    ty = limelightTable.getEntry("ty");
    ta = limelightTable.getEntry("ta");
    tv = limelightTable.getEntry("tv");
  }

  @Override
  public void periodic() {
    updateInformation();
  }

  @Override
  public void disable() {}

  private void updateInformation() {
    horizontalTargetAngle = tx.getDouble(0.0);
    verticalTargetAngle = ty.getDouble(0.0);
    targetArea = ta.getDouble(0.0);
    targetValid = tv.getBoolean(false);
    calculateDistance();
  }

  private void calculateDistance() {
    targetDistance =
      (
        (config.vision.targetHeight - config.vision.cameraHeight) /
        Math.tan(config.vision.verticalCameraAngle + verticalTargetAngle)
      );
  }

  public double getDistance() {
    return targetDistance;
  }

  public double getVerticalAngle() {
    return verticalTargetAngle;
  }

  public double getAbsVerticalAngle() {
    return verticalTargetAngle + config.vision.verticalCameraAngle;
  }

  public double getHorizontalAngle() {
    return horizontalTargetAngle;
  }
}
