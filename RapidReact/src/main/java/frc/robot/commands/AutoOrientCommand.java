package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.DrivetrainSubsystem;
import frc.robot.subsystem.VisionSubsystem;

import java.util.function.DoubleSupplier;

public class AutoOrientCommand extends CommandBase {

    final DoubleSupplier x;
    final DoubleSupplier y;
    final DrivetrainSubsystem drivetrainSubsystem;
    final VisionSubsystem visionSubsystem;
    final ProfiledPIDController rotControllerRad;

    public AutoOrientCommand(DoubleSupplier x, DoubleSupplier y, DrivetrainSubsystem drivetrainSubsystem, VisionSubsystem visionSubsystem) {
        this.x = x;
        this.y = y;
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.visionSubsystem = visionSubsystem;

        rotControllerRad = new ProfiledPIDController(
                3,
                0.2,
                0.02,
                new TrapezoidProfile.Constraints(
                        drivetrainSubsystem.getMaxAngularVelocity(),
                        drivetrainSubsystem.getMaxAngularVelocity() * 10.0
                )
        );

        addRequirements(drivetrainSubsystem);
    }


    @Override
    public void execute() {
        Rotation2d gyroRot = drivetrainSubsystem.getGyroAngle();

        double offsetRotDeg = visionSubsystem.getHeadingOffsetAngle();
        double errorDeg = Math.abs(offsetRotDeg);

        //first deadband
        Rotation2d target = gyroRot;
        if (errorDeg > 1.0) {
            if (offsetRotDeg > 0) { //offset is positive, subtract
                target = gyroRot.minus(Rotation2d.fromDegrees(errorDeg));
            } else { //offset is negative, add
                target = gyroRot.plus(Rotation2d.fromDegrees(errorDeg));
            }
        }

        double rotationOutput = rotControllerRad.calculate(
                gyroRot.getRadians(),
                target.getRadians()
        );

        //another deadband
        if (Math.abs(rotationOutput) < 0.05) {
            rotationOutput = 0.0; //deadband that shit
        }

        drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(x.getAsDouble(), y.getAsDouble(), rotationOutput, drivetrainSubsystem.getGyroAngle())
        );
    }
}
