package frc.robot.subsystem;

import edu.wpi.first.hal.HAL;
import frc.robot.config.Config;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class IntakeSubsystemTest extends SubsystemTest {

    public static final double DELTA = 0.1; // acceptable deviation range
    IntakeSubsystem subsystem;

    @Before
    public void setup() {
        assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
        Config config = new Config();

        // create the subsystem
        subsystem = new IntakeSubsystem(config);
        subsystem.init();

        // set the bus voltage to the motor so our output is correct
        subsystem.intake.getSimCollection().setBusVoltage(BUS_VOLTAGE);
    }

    @Test
    public void testSpinForward() {

        // spin the intake motor
        subsystem.spinForward();



        // wait for the CTRE sim to update (weird)
        waitForCTREUpdate();

        // verify we applied 66% of the bus voltage to the motor (this is the default speed)
        assertEquals(-0.60 * 11.5, subsystem.intake.getSimCollection().getMotorOutputLeadVoltage(), DELTA);
    }

}
