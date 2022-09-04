package frc.robot.lib.motors;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.swervedrivespecialties.swervelib.SwerveModule;

public interface Motors {

    MotorBuilder<TalonFX> talonFX(int port);
    MotorBuilder<TalonSRX> talonSRX(int port);

    //TODO use l2 or allow using other ratio
    SwerveModule falconSwerveModule(int driveMotorPort, int steerMotorPort, int steerEncoderPort, double steerOffset);

}
