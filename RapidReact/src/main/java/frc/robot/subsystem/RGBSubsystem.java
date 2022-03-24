package frc.robot.subsystem;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import frc.robot.config.Config;
import frc.robot.log.BucketLog;
import frc.robot.log.Changeable;
import frc.robot.log.Put;
import frc.robot.utils.BlinkenColors;

public class RGBSubsystem extends BitBucketsSubsystem {

  private final PWMSparkMax motor;

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

  public void funnyButton() {
    motor.set(BlinkenColors.Rainbow_Party_Palette.getValue());
  }

  public void normalize() {
    motor.set(BlinkenColors.Colors_Violet.getValue());
  }

  public void autoDriving()
  {
    motor.set(BlinkenColors.Fire_Medium.getValue());
  }

  public void autoNotDriving()
  {
    motor.set(BlinkenColors.Twinkles_Party_Palette.getValue());
  }

  public void upToSpeed() {
    motor.set(BlinkenColors.Colors_Green.getValue());  
  }
  
  public void notUpToSpeed() {
    motor.set(BlinkenColors.Colors_Red.getValue());  
  }

  public void autoShootingSingle()
  {
    motor.set(BlinkenColors.Strobe_White.getValue());
  }

  public void autoShootingDouble()
  {
    motor.set(BlinkenColors.Strobe_Gold.getValue());
  }

  public void alertMatchTimeLeft()
  {
    motor.set(BlinkenColors.Colors_Yellow.getValue());
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
