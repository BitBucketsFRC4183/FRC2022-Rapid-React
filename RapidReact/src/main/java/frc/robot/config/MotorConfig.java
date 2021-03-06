package frc.robot.config;

public class MotorConfig {

  //////////////////////////////////////////////////////////////////////////////
  // Other
  public enum EncoderType {
    // Again, you already know what this one does, I'm just writing this so that
    // VSCode doesn't mess up my format.
    None,

    // Starts at 0, increments ticks based on how much they rotated.
    Quadrature,

    // Effectively the same thing as quadrature, there might be a few differences,
    // but I can't think of any.
    Relative,

    // Whereas relative and quadrature determine their position by the amount they
    // have rotated, absolute determines where it is based on the angle of the
    // motor.
    Absolute,

    // Uses the internal sensor if the motor has one.
    Integrated,
  }

  public EncoderType encoderType = EncoderType.Quadrature;

  public int ticksPerRevolution;

  public boolean inverted;

  public boolean sensorPhase;

  public int motionMagicAcceleration;

  public int motionMagicCruiseVelocity;

  public double distancePeakOutput;
  public double turningPeakOutput;

  //////////////////////////////////////////////////////////////////////////////
  // PIDFs

  public PIDF velocityPIDF = new PIDF();

  public PIDF positionPIDF = new PIDF();

  //////////////////////////////////////////////////////////////////////////////
  // ID

  public int id;

  public int followingID = -1;
}
