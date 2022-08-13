package frc.robot.subsystem.other;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.Config;

public abstract class BitBucketsSubsystem extends SubsystemBase {

  protected final Config config;

  protected BitBucketsSubsystem(Config config) {
    super();
    this.setName(this.getClass().getSimpleName());

    this.config = config;
  }

  //When the subsystem is initialized
  public abstract void init();

  //Periodically called
  public abstract void periodic();

  public abstract void disable();
}
