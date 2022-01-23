package frc.swervelib;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.config.Config;
import frc.robot.subsystem.DrivetrainSubsystem;
import frc.wpiClasses.QuadSwerveSim;
import frc.wpiClasses.SwerveModuleSim;
import java.util.ArrayList;

public class SwerveDrivetrainModel {

  QuadSwerveSim swerveDt;
  ArrayList<SwerveModule> realModules;
  ArrayList<SwerveModuleSim> simModules = new ArrayList<>(QuadSwerveSim.NUM_MODULES);

  Field2d field;
  Gyroscope gyro;

  SwerveDrivePoseEstimator poseEstimator;

  SwerveDriveKinematics kinematics;

  Pose2d curEstPose;
  SwerveModuleState[] states;
  ProfiledPIDController thetaController = new ProfiledPIDController(
    SwerveConstants.THETACONTROLLERkP,
    0,
    0,
    SwerveConstants.THETACONTROLLERCONSTRAINTS
  );

  public SwerveDrivetrainModel(ArrayList<SwerveModule> realModules, Gyroscope gyro, SwerveDriveKinematics _kinematics, Field2d field, Pose2d startPosition) {
    this.kinematics = _kinematics;
    this.gyro = gyro;
    this.realModules = realModules;
    this.field = field;

    if (RobotBase.isSimulation()) {
      simModules.add(Mk4SwerveModuleHelper.createSim(realModules.get(0)));
      simModules.add(Mk4SwerveModuleHelper.createSim(realModules.get(1)));
      simModules.add(Mk4SwerveModuleHelper.createSim(realModules.get(2)));
      simModules.add(Mk4SwerveModuleHelper.createSim(realModules.get(3)));
    }

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // TODO: move these to config
    double mass_kg = Units.lbsToKilograms(140);
    double trackWidth_m = .75;
    double trackLength_m = .75;
    double moi = 1.0/12.0 * mass_kg * Math.pow((trackWidth_m*1.1),2) * 2;
    swerveDt =
      new QuadSwerveSim(
        trackWidth_m, // SwerveConstants.TRACKWIDTH_METERS,
        trackLength_m, // SwerveConstants.TRACKLENGTH_METERS,
        mass_kg, // SwerveConstants.MASS_kg,
        moi, //Model moment of intertia as a square slab slightly bigger than wheelbase with axis through center, // SwerveConstants.MOI_KGM2,
        simModules
      );

    // Trustworthiness of the internal model of how motors should be moving
    // Measured in expected standard deviation (meters of position and degrees of
    // rotation)
    var stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));

    // Trustworthiness of gyro in radians of standard deviation.
    var localMeasurementStdDevs = VecBuilder.fill(Units.degreesToRadians(0.1));

    // Trustworthiness of the vision system
    // Measured in expected standard deviation (meters of position and degrees of
    // rotation)
    var visionMeasurementStdDevs = VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(0.1));


    curEstPose = new Pose2d(startPosition.getTranslation(), startPosition.getRotation());
    poseEstimator =
      new SwerveDrivePoseEstimator(
        getGyroscopeRotation(),
        startPosition,
        kinematics,
        stateStdDevs,
        localMeasurementStdDevs,
        visionMeasurementStdDevs,
        SimConstants.CTRLS_SAMPLE_RATE_SEC
      );

    setKnownPose(startPosition);

  }

  /**
   * Handles discontinuous jumps in robot pose. Used at the start of
   * autonomous, if the user manually drags the robot across the field in the
   * Field2d widget, or something similar to that.
   * @param pose
   */
  public void modelReset(Pose2d pose) {
    swerveDt.modelReset(pose);
    gyro.setAngle(pose.getRotation());
  }

  /**
   * Advance the simulation forward by one step
   * @param isDisabled
   * @param batteryVoltage
   */
  public void update(boolean isDisabled, double batteryVoltage) {
    // Calculate and update input voltages to each motor.
    if (isDisabled) {
      for (int idx = 0; idx < QuadSwerveSim.NUM_MODULES; idx++) {
        simModules.get(idx).setInputVoltages(0.0, 0.0);
      }
    } else {
      for (int idx = 0; idx < QuadSwerveSim.NUM_MODULES; idx++) {
        double steerVolts = realModules.get(idx).getSteerController().getOutputVoltage();
        double wheelVolts = realModules.get(idx).getDriveController().getOutputVoltage();
        simModules.get(idx).setInputVoltages(wheelVolts, steerVolts);
      }
    }

    //Update the main drivetrain plant model
    swerveDt.update(SimConstants.SIM_SAMPLE_RATE_SEC);

    // Update each encoder
    for (int idx = 0; idx < QuadSwerveSim.NUM_MODULES; idx++) {
      double azmthShaftPos = simModules.get(idx).getAzimuthEncoderPositionRev();
      double steerMotorPos = simModules.get(idx).getAzimuthMotorPositionRev();
      double wheelPos = simModules.get(idx).getWheelEncoderPositionRev();

      double azmthShaftVel = simModules.get(idx).getAzimuthEncoderVelocityRPM();
      double steerVelocity = simModules.get(idx).getAzimuthMotorVelocityRPM();
      double wheelVelocity = simModules.get(idx).getWheelEncoderVelocityRPM();

      realModules.get(idx).getAbsoluteEncoder().setAbsoluteEncoder(azmthShaftPos, azmthShaftVel);
      realModules.get(idx).getSteerController().setSteerEncoder(steerMotorPos, steerVelocity);
      realModules.get(idx).getDriveController().setDriveEncoder(wheelPos, wheelVelocity);
    }

    // Update associated devices based on drivetrain motion
    gyro.setAngle(swerveDt.getCurPose().getRotation());

    // add swerve modules to field
    field.getObject("Simulated Robot").setPose(swerveDt.getCurPose());
    field.getObject("Swerve Modules").setPoses(swerveDt.getModulePoses());

  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param chassisSpeeds The desired SwerveModule states.
   */
  public void setModuleStates(ChassisSpeeds chassisSpeeds) {
    this.states = kinematics.toSwerveModuleStates(chassisSpeeds);
  }

  public SwerveModuleState[] getSwerveModuleStates() {
    return states;
  }

  public void setKnownPose(Pose2d in) {
    resetWheelEncoders();
    // No need to reset gyro, pose estimator does that.
    poseEstimator.resetPosition(in, getGyroscopeRotation());
    curEstPose = in;
  }

  public Rotation2d getGyroscopeRotation() {
    return gyro.getGyroHeading();
  }

  public void resetWheelEncoders() {
    for (int idx = 0; idx < QuadSwerveSim.NUM_MODULES; idx++) {
      realModules.get(idx).resetWheelEncoder();
    }
  }

  public Command createCommandForTrajectory(PathPlannerTrajectory trajectory, DrivetrainSubsystem m_drive, SwerveDriveKinematics kinematics) {
    SwerveControllerCommandPP swerveControllerCommand = new SwerveControllerCommandPP(
      trajectory,
      // TODO: is this curEstPose correct? Not sure if this is set outside of the sim
      () -> curEstPose, // Functional interface to feed supplier
      kinematics,
      // Position controllers
      SwerveConstants.XPIDCONTROLLER,
      SwerveConstants.YPIDCONTROLLER,
      thetaController,
      commandStates -> this.states = commandStates,
      m_drive
    );
    return swerveControllerCommand;
  }

  public ArrayList<SwerveModule> getRealModules() {
    return realModules;
  }

  public ArrayList<SwerveModuleSim> getSimModules() {
    return simModules;
  }

}
