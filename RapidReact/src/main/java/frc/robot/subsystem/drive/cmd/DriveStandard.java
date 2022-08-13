package frc.robot.subsystem.drive.cmd;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.drive.DriveSubsystem;

import static frc.robot.subsystem.drive.DriveConstants.*;

public class DriveStandard extends CommandBase {

    private final DriveSubsystem driveSubsystem;


    private final SlewRateLimiter[] limiter = new SlewRateLimiter[] { new SlewRateLimiter(2), new SlewRateLimiter(2) };
    private final Joystick joystick = new Joystick(1);

    public DriveStandard(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double x = limiter[AXIS_X].calculate(joystick.getRawAxis(AXIS_X));
        double y = limiter[AXIS_Y].calculate(joystick.getRawAxis(AXIS_Y));
        double rot = joystick.getRawAxis(AXIS_ROT);


        //TODO profile for "slowing down" values so we can prematurely lock, dunno if we need it or not

        if (x == 0 && y == 0 && rot == 0) {

            ChassisSpeeds setLock = new ChassisSpeeds(0,0,0);



            driveSubsystem.command();

            driveSubsystem.stopSticky();
            xWheelLoggable.log(LogLevel.DEBUG, "xWheel active.");
        } else {
            driveSubsystem.drive(
                    ChassisSpeeds.fromFieldRelativeSpeeds(xOutput, yOutput, rotationOutput, driveSubsystem.getGyroAngle())
            );
            xWheelLoggable.log(LogLevel.DEBUG, "xWheel inactive.");
        }


    }
}
