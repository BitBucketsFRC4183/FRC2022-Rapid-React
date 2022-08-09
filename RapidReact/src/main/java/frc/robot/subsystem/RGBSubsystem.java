package frc.robot.subsystem;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import frc.robot.config.Config;
import frc.robot.lib.RunCycle;
import frc.robot.utils.BlinkenColors;

public class RGBSubsystem implements RunCycle {

  private final PWMSparkMax motor = new PWMSparkMax(Config.RGB_ID);

  public void setColor(BlinkenColors color) {
    motor.set(color.getValue());
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
    motor.set(BlinkenColors.Fire_Large.getValue());
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
    motor.set(BlinkenColors.Color_Waves_Lava_Palette.getValue());
  }

  public void alertMatchTimeLeft()
  {
    motor.set(BlinkenColors.Colors_Yellow.getValue());
  }

  @Override
  public void init() {

  }

  @Override
  public void periodic() {
    // TODO Auto-generated method stub

  }

  @Override
  public void stop() {

  }



}
