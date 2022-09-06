package frc.robot.drive.command;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.drive.Drive;
import frc.robot.drive.DriveSystem;
import frc.robot.drive.odometry.OdometrySystem;
import frc.robot.lib.System;
import frc.robot.lib.exec.Pressable;
import frc.robot.lib.header.SystemContext;
import frc.robot.utils.MathUtils;
import frc.robot.vision.VisionSystem;

import static frc.robot.utils.ArrayConstants.*;

public class DriveDirectorTeleop implements System {

    final DriveSystem driveSystem;
    final OdometrySystem odometrySystem;
    final VisionSystem visionSystem;

    public DriveDirectorTeleop(SystemContext context, DriveSystem driveSystem, OdometrySystem odometrySystem, VisionSystem visionSystem) {
        this.driveSystem = driveSystem;
        this.odometrySystem = odometrySystem;
        this.visionSystem = visionSystem;

        slowDrive = context.buttons().SLOW_DRIVE;
        aimDrive = context.buttons().AIM_DRIVE;

        driverControl = context.buttons().driverControl;

        double maxAngVel = Drive.MAX_ANGULAR_VEL;
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(maxAngVel, maxAngVel * 10.0);

        rotControllerRad = new ProfiledPIDController(3, 0.2, 0.02, constraints);

    }

    final SlewRateLimiter[] limiter = new SlewRateLimiter[] { new SlewRateLimiter(2), new SlewRateLimiter(2) };

    final Pressable slowDrive;
    final Pressable aimDrive;
    final Joystick driverControl;
    final ProfiledPIDController rotControllerRad;


    @Override
    public void periodic(float delta, int iteration) {

        double x = limiter[AXIS_X].calculate(MathUtils.modifyAxis(driverControl.getRawAxis(AXIS_X)));
        double y = limiter[AXIS_Y].calculate(MathUtils.modifyAxis(driverControl.getRawAxis(AXIS_Y)));
        double rot = MathUtils.modifyAxis(driverControl.getRawAxis(AXIS_ROT));

        boolean aim = aimDrive.held();
        double coefficient = slowDrive.held() ? 0.25 : 1;

        if (aim) {
            driveOrient(x, y, rot, coefficient);
        } else {
            if (x == 0 && y == 0 && rot == 0) {
                lock();
            } else {
                drive(x, y, rot, coefficient);
            }
        }

    }

    public void lock() {
        driveSystem.driveAt(
                new SwerveModuleState[]{
                        new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                        new SwerveModuleState(0, Rotation2d.fromDegrees(45))
                }
        );
    }

    public void drive(double x, double y, double rot, double gain) {
        driveSystem.driveAt(
                odometrySystem.calculateFieldOrientedDesiredStates(x, y, rot, gain)
        );
    }

    public void driveOrient(double x, double y, double joystickRot, double gain) {

        Rotation2d gyroRot = odometrySystem.rotationGyro();

        double offsetRotDeg = visionSystem.getHeadingOffsetDeg();
        double errorDeg = Math.abs(offsetRotDeg);

        //first deadband
        Rotation2d target = new Rotation2d(joystickRot);
        if (errorDeg > Drive.VISION_DRIVE_THRESHOLD_DEG) {
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

        drive(x, y, rotationOutput, gain);
    }
}
