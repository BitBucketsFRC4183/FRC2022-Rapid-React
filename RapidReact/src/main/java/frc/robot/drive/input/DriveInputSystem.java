package frc.robot.drive.input;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import frc.model.drive.DriveInput;
import frc.robot.lib.System;
import frc.robot.lib.exec.Pressable;

public class DriveInputSystem implements DriveInput {

    final Joystick joystick;
    final Pressable slow;
    final Pressable autoAim;

    public DriveInputSystem(Joystick joystick, Pressable slow, Pressable autoAim) {
        this.joystick = joystick;
        this.slow = slow;
        this.autoAim = autoAim;
    }

    @Override
    public double normalizedX() {
       return DriveInput.X_LIMITER.calculate(
               modifyAxis(
                       joystick.getRawAxis(0)
               )
       );
    }

    @Override
    public double normalizedY() {
        return DriveInput.Y_LIMITER.calculate(
                modifyAxis(
                        joystick.getRawAxis(1)
                )
        );
    }

    @Override
    public double rotation() {
        return modifyAxis(
                joystick.getRawAxis(2)
        );
    }

    @Override
    public boolean shouldSlowDrive() {
        return slow.held();
    }

    @Override
    public boolean shouldAimDrive() {
        return slow.held();
    }

    double modifyAxis(double value) {
        // Deadband
        value = MathUtil.applyDeadband(value, 0.1);

        // Square the axis
        value = Math.copySign(value * value, value);

        return value;
    }




}
