package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.DrivetrainSubsystem;
import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {

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

    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void execute() {
    // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
    ChassisSpeeds robotOrient = new ChassisSpeeds(
      translationXSupplier.getAsDouble() * driveSubsystem.maxVelocity_metersPerSecond / 2,
      translationYSupplier.getAsDouble() * driveSubsystem.maxVelocity_metersPerSecond / 2,
      rotationSupplier.getAsDouble() * driveSubsystem.maxAngularVelocity_radiansPerSecond / 2
    );
    driveSubsystem.drive(robotOrient);
    /* m_drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        m_translationXSupplier.getAsDouble(),
                        m_translationYSupplier.getAsDouble(),
                        m_rotationSupplier.getAsDouble(),
                        m_drivetrainSubsystem.getGyroscopeRotation())
                
        );*/
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }
}
