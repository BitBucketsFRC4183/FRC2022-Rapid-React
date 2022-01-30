package frc.swervelib;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.swervelib.ctre.*;
import frc.wpiClasses.QuadSwerveSim;
import frc.wpiClasses.SwerveModuleSim;

public final class Mk4SwerveModuleHelper {

  private Mk4SwerveModuleHelper() {}

  private static DriveControllerFactory<?, Integer> getFalcon500DriveFactory(Mk4ModuleConfiguration configuration) {
    return new Falcon500DriveControllerFactoryBuilder()
      .withVoltageCompensation(configuration.getNominalVoltage())
      .withCurrentLimit(configuration.getDriveCurrentLimit())
      .build();
  }

  private static SteerControllerFactory<?, Falcon500SteerConfiguration<CanCoderAbsoluteConfiguration>> getFalcon500SteerFactory(
    Mk4ModuleConfiguration configuration
  ) {
    return new Falcon500SteerControllerFactoryBuilder()
      .withVoltageCompensation(configuration.getNominalVoltage())
      .withPidConstants(0.2, 0.0, 0.1)
      .withCurrentLimit(configuration.getSteerCurrentLimit())
      .build(new CanCoderFactoryBuilder().withReadingUpdatePeriod(100).build());
  }

  /**
   * Creates a Mk4 swerve module that uses Falcon 500s for driving and steering.
   * Module information is displayed in the specified ShuffleBoard container.
   *
   * @param container        The container to display module information in.
   * @param configuration    Module configuration parameters to use.
   * @param gearRatio        The gearing configuration the module is in.
   * @param driveMotorPort   The CAN ID of the drive Falcon 500.
   * @param steerMotorPort   The CAN ID of the steer Falcon 500.
   * @param steerEncoderPort The CAN ID of the steer CANCoder.
   * @param steerOffset      The offset of the CANCoder in radians.
   * @return The configured swerve module.
   */
  public static SwerveModule createFalcon500(
    ShuffleboardLayout container,
    Mk4ModuleConfiguration configuration,
    GearRatio gearRatio,
    int driveMotorPort,
    int steerMotorPort,
    int steerEncoderPort,
    double steerOffset,
    String namePrefix
  ) {
    return new SwerveModuleFactory<>(
      gearRatio.getConfiguration(),
      getFalcon500DriveFactory(configuration),
      getFalcon500SteerFactory(configuration)
    )
    .create(
        container,
        driveMotorPort,
        new Falcon500SteerConfiguration<>(
          steerMotorPort,
          new CanCoderAbsoluteConfiguration(steerEncoderPort, steerOffset)
        ),
        namePrefix
      );
  }

  /**
   * Creates a Mk4 swerve module that uses Falcon 500s for driving and steering.
   * Module information is displayed in the specified ShuffleBoard container.
   *
   * @param container        The container to display module information in.
   * @param gearRatio        The gearing configuration the module is in.
   * @param driveMotorPort   The CAN ID of the drive Falcon 500.
   * @param steerMotorPort   The CAN ID of the steer Falcon 500.
   * @param steerEncoderPort The CAN ID of the steer CANCoder.
   * @param steerOffset      The offset of the CANCoder in radians.
   * @return The configured swerve module.
   */
  public static SwerveModule createFalcon500(
    ShuffleboardLayout container,
    GearRatio gearRatio,
    int driveMotorPort,
    int steerMotorPort,
    int steerEncoderPort,
    double steerOffset,
    String namePrefix
  ) {
    return createFalcon500(
      container,
      new Mk4ModuleConfiguration(),
      gearRatio,
      driveMotorPort,
      steerMotorPort,
      steerEncoderPort,
      steerOffset,
      namePrefix
    );
  }

  /**
   * Creates a Mk4 swerve module that uses Falcon 500s for driving and steering.
   *
   * @param configuration    Module configuration parameters to use.
   * @param gearRatio        The gearing configuration the module is in.
   * @param driveMotorPort   The CAN ID of the drive Falcon 500.
   * @param steerMotorPort   The CAN ID of the steer Falcon 500.
   * @param steerEncoderPort The CAN ID of the steer CANCoder.
   * @param steerOffset      The offset of the CANCoder in radians.
   * @return The configured swerve module.
   */
  public static SwerveModule createFalcon500(
    Mk4ModuleConfiguration configuration,
    GearRatio gearRatio,
    int driveMotorPort,
    int steerMotorPort,
    int steerEncoderPort,
    double steerOffset,
    String namePrefix
  ) {
    return new SwerveModuleFactory<>(
      gearRatio.getConfiguration(),
      getFalcon500DriveFactory(configuration),
      getFalcon500SteerFactory(configuration)
    )
    .create(
        driveMotorPort,
        new Falcon500SteerConfiguration<>(
          steerMotorPort,
          new CanCoderAbsoluteConfiguration(steerEncoderPort, steerOffset)
        ),
        namePrefix
      );
  }

  /**
   * Creates a Mk4 swerve module that uses Falcon 500s for driving and steering.
   *
   * @param gearRatio        The gearing configuration the module is in.
   * @param driveMotorPort   The CAN ID of the drive Falcon 500.
   * @param steerMotorPort   The CAN ID of the steer Falcon 500.
   * @param steerEncoderPort The CAN ID of the steer CANCoder.
   * @param steerOffset      The offset of the CANCoder in radians.
   * @return The configured swerve module.
   */
  public static SwerveModule createFalcon500(
    GearRatio gearRatio,
    int driveMotorPort,
    int steerMotorPort,
    int steerEncoderPort,
    double steerOffset,
    String namePrefix
  ) {
    return createFalcon500(
      new Mk4ModuleConfiguration(),
      gearRatio,
      driveMotorPort,
      steerMotorPort,
      steerEncoderPort,
      steerOffset,
      namePrefix
    );
  }

  public enum GearRatio {
    L1(SdsModuleConfigurations.MK4_L1),
    L2(SdsModuleConfigurations.MK4_L2),
    L3(SdsModuleConfigurations.MK4_L3),
    L4(SdsModuleConfigurations.MK4_L4);

    private final ModuleConfiguration configuration;

    GearRatio(ModuleConfiguration configuration) {
      this.configuration = configuration;
    }

    public ModuleConfiguration getConfiguration() {
      return configuration;
    }
  }

  public static SwerveModuleSim createSim(SwerveModule module) {
    // TODO: use config
    double mass_kg = Units.lbsToKilograms(140);
    ModuleConfiguration modConfig = module.getModuleConfiguration();
    return new SwerveModuleSim(
      module.getSteerController().getSteerMotor(),
      module.getDriveController().getDriveMotor(),
      modConfig.getWheelDiameter() / 2,
      1 / modConfig.getSteerReduction(),
      1 / modConfig.getDriveReduction(),
      1.0, // CANCoder is directly on the shaft
      1 / modConfig.getDriveReduction(),
      1.1,
      0.8,
      mass_kg * 9.81 / QuadSwerveSim.NUM_MODULES,
      0.01
    );
  }
}
