package frc.robot.bs;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.*;

public class Intake implements AutoCloseable {
    private TalonSRX motor;
    private DoubleSolenoid piston;

    public Intake() {
        motor = new TalonSRX(2);
        piston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    }

    public void deploy() {
        piston.set(DoubleSolenoid.Value.kForward);
    }

    public void retract() {
        piston.set(DoubleSolenoid.Value.kReverse);

        motor.set(ControlMode.PercentOutput, 0);
    }

    public void activate(double speed) {
        if (piston.get() == DoubleSolenoid.Value.kForward) {
            motor.set(ControlMode.PercentOutput, speed);
        } else { // if piston isn't open, do nothing
            motor.set(ControlMode.PercentOutput, 0);
        }
    }

    @Override
    public void close() throws Exception {
        piston.close();
    }
}