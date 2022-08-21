package frc.robot.subsystem.drive;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.lib.exec.CommandState;

import static frc.robot.utils.ArrayConstants.*;

public class DriveCommands {

    final SlewRateLimiter[] limiter = new SlewRateLimiter[] { new SlewRateLimiter(2), new SlewRateLimiter(2) };
    final Joystick joystick = new Joystick(1);
    final DriveSystem driveSystem;

    public DriveCommands(DriveSystem driveSystem) {
        this.driveSystem = driveSystem;
    }

    public boolean slowToggle(CommandState state) {
        if (state == CommandState.INIT) {
            driveSystem.slowMode();
        }

        return true;
    }

    public boolean driveStandard(CommandState state) {
        if (state != CommandState.NORMAL) return false;

        double x = limiter[AXIS_X].calculate(joystick.getRawAxis(AXIS_X));
        double y = limiter[AXIS_Y].calculate(joystick.getRawAxis(AXIS_Y));
        double rot = joystick.getRawAxis(AXIS_ROT);

        if (x == 0 && y == 0 && rot == 0) {

            driveSystem.driveAt( //x lock!
                    new SwerveModuleState[]{
                            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                            new SwerveModuleState(0, Rotation2d.fromDegrees(45))
                    }
            );

        } else {
            driveSystem.driveFieldOriented(x,y,rot);
        }

        return false;
    }
}
