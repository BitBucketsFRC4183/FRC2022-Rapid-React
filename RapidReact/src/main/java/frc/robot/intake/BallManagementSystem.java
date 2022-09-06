package frc.robot.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.lib.System;

public class BallManagementSystem implements System {

    final WPI_TalonSRX ballManagement;

    public BallManagementSystem(WPI_TalonSRX ballManagement) {
        this.ballManagement = ballManagement;
    }




    public void bringBallsIn() {
        ballManagement.set(ControlMode.PercentOutput, BallManagement.BMS_FEED.get());
    }

    public void pushBallsOut() {
        ballManagement.set(ControlMode.PercentOutput, -BallManagement.BMS_FEED.get());
    }

    public void resetMotor() {
        ballManagement.set(ControlMode.PercentOutput, 0);
    }
}
