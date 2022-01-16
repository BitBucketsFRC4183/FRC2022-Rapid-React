package frc.robot.bs;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

public class IntakeTest {
    public static final double DELTA = 1e-2; // acceptable deviation range
    Intake intake;
    PWMSim simMotor;

    @Before // this method will run before each test
    public void setup() {
        assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
        intake = new Intake(); // create our intake
        simMotor = new PWMSim(2); // create our simulation PWM motor controller
    }

    @After // this method will run after each test
    public void shutdown() throws Exception {
        intake.close(); // destroy our intake object
    }

    @Test // marks this method as a test
    public void doesntWorkWhenClosed() {
        intake.retract(); // close the intake
        intake.activate(0.5); // try to activate the motor
    }

    @Test
    public void worksWhenOpen() {
        intake.deploy();
        intake.activate(0.5);
    }

    @Test
    public void retractTest() {
        intake.retract();
    }

    @Test
    public void deployTest() {
        intake.deploy();
    }
}