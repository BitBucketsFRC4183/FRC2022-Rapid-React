package frc.robot.subsystem;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import frc.robot.config.Config;
import frc.robot.config.Config;
import frc.robot.log.BucketLog;
import frc.robot.log.Changeable;
import frc.robot.log.Put;
import frc.robot.utils.BlinkenColors;

public class RGBSubsystem extends BitBucketsSubsystem {

  private double climbEnabled = 0.59;
  private double autoClimb = 0.57;
  private PWMSparkMax motor;

  private final Changeable<Double> color = BucketLog.changeable(
    Put.DOUBLE,
    "rgb/color",
    BlinkenColors.Colors_Violet.getValue()
  );

  public RGBSubsystem(Config config) {
    super(config);
    motor = new PWMSparkMax(Config.RGB_ID);
    //TODO Auto-generated constructor stub
  }

  @Override
  public void init() {
    // TODO Auto-generated method stub
    motor.set(color.currentValue());
  }

  public void climberEnabled() {
    motor.set(BlinkenColors.Colors_Dark_red.getValue());
  }

  public void autoClimb() {
    motor.set(BlinkenColors.Colors_Hot_Pink.getValue());
  }

  public void colorChange() {

  }

  public void funnyButton() {
    motor.set(BlinkenColors.Rainbow_Party_Palette.getValue());
  }

  @Override
  public void periodic() {
    // TODO Auto-generated method stub

  }

  @Override
  public void disable() {
    // TODO Auto-generated method stub

  }
}
