package frc.robot.subsystem.drive;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.RunCycle;

import static com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper.GearRatio.L2;
import static frc.robot.subsystem.drive.DriveConstants.*;

public class DriveSubsystem extends SubsystemBase implements RunCycle {

    final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(0.65292, 2.3053, 0.37626);


    //identical
    final SwerveModule[] modules = new SwerveModule[4];


    double speedModifier = 0.75;
    AHRS gyro;
    SwerveDriveOdometry odometry;


    @Override
    public void init() {
        PIDController controller;
        //instantiate
        gyro = new AHRS(SPI.Port.kMXP, (byte)200);
        odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d()); //this rot2d will update in periodic

        ShuffleboardTab tab = Shuffleboard.getTab("drivetrain");

        ShuffleboardLayout frontLeft = tab
                .getLayout("Front Left Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(0, 0);
        ShuffleboardLayout frontRight = tab
                .getLayout("Front Right Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(2, 0);
        ShuffleboardLayout backLeft = tab
                .getLayout("Back Left Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(4, 0);
        ShuffleboardLayout backRight = tab
                .getLayout("Back Right Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(6, 0);

        //intellij highlights exactly what each parameter does here, meaning we don't need config to tell us
        modules[0] = Mk4SwerveModuleHelper.createFalcon500(frontLeft, L2, 1, 2, 9, -Math.toRadians(232.55));
        modules[1] = Mk4SwerveModuleHelper.createFalcon500(frontRight, L2, 7, 8, 12, -Math.toRadians(331.96 - 180));
        modules[2] = Mk4SwerveModuleHelper.createFalcon500(backLeft, L2, 5, 6, 11, -Math.toRadians(255.49));
        modules[3] = Mk4SwerveModuleHelper.createFalcon500(backRight, L2, 3, 4, 10, -Math.toRadians(70.66 + 180));

        gyro.calibrate();
        odometry.resetPosition(new Pose2d(), gyro.getRotation2d());
    }

    @Override
    public void periodic(float delta) {
        SwerveModuleState[] states = new SwerveModuleState[4];

        for (int i = 0; i < modules.length; i++) {
            SwerveModule module = modules[i];

            states[i] = new SwerveModuleState(module.getDriveVelocity(), new Rotation2d(module.getSteerAngle()));
        }

        this.odometry.update(gyro.getRotation2d(), states);
    }

    @Override
    public void stop() {
        command(new ChassisSpeeds(0,0,0));
    }


    public void command(SwerveModuleState[] states) {
        for(int i = 0; i < modules.length; i++)
        {
            double voltage = MathUtil.clamp(feedForward.calculate(states[i].speedMetersPerSecond), -12, 12);
            double radians = states[i].angle.getRadians();

            modules[i].set(voltage, radians);
        }
    }

    /**
     * Commands the robot to move to the velocities specified
     * @param speeds speeds
     */
    public void command(ChassisSpeeds speeds) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VEL_METERS_PER_SECOND * speedModifier);




    }

    public void resetOdometer() {
        odometry.resetPosition(new Pose2d(), new Rotation2d());
    }

    public void speedModifier(double speed) {
        this.speedModifier = speed;
    }

}
