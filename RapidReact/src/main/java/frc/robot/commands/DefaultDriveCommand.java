package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.DrivetrainSubsystem;
import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {

  private static final SendableChooser<String> orientationChooser = new SendableChooser<>();

  private final DrivetrainSubsystem driveSubsystem;

  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;
  private final DoubleSupplier rotationSupplier;

  public DefaultDriveCommand(
    DrivetrainSubsystem drivetrainSubsystem,
    DoubleSupplier translationXSupplier,
    DoubleSupplier translationYSupplier,
    DoubleSupplier rotationSupplier
  ) {
    this.driveSubsystem = drivetrainSubsystem;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.rotationSupplier = rotationSupplier;
    // Control Orientation Chooser
    orientationChooser.setDefaultOption("Field Oriented", "Field Oriented");
    orientationChooser.addOption("Robot Oriented", "Robot Oriented");
    SmartDashboard.putData("Orientation Chooser", orientationChooser);
    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void execute() {
    // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
    switch (orientationChooser.getSelected()) {
      case "Field Oriented":
        driveSubsystem.drive(
          ChassisSpeeds.fromFieldRelativeSpeeds(
            translationXSupplier.getAsDouble() * driveSubsystem.maxVelocity_metersPerSecond,
            translationYSupplier.getAsDouble() * driveSubsystem.maxVelocity_metersPerSecond,
            rotationSupplier.getAsDouble() * driveSubsystem.maxAngularVelocity_radiansPerSecond,
            driveSubsystem.getGyroscopeRotation()
          )
        );
        break;
      case "Robot Oriented":
        ChassisSpeeds robotOrient = new ChassisSpeeds(
          translationXSupplier.getAsDouble() * driveSubsystem.maxVelocity_metersPerSecond,
          translationYSupplier.getAsDouble() * driveSubsystem.maxVelocity_metersPerSecond,
          rotationSupplier.getAsDouble() * driveSubsystem.maxAngularVelocity_radiansPerSecond
        );
        driveSubsystem.drive(robotOrient);
        break;
    }
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }
}
