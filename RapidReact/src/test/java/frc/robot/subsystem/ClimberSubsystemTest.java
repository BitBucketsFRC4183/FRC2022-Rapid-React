package frc.robot.subsystem;

import edu.wpi.first.hal.HAL;
import frc.robot.config.Config;
import frc.robot.config.PIDF;
import frc.robot.simulator.CTREPhysicsSim;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.*;

public class ClimberSubsystemTest extends SubsystemTest {

    public static final double DELTA = 1e-2; // acceptable deviation range
    ClimberSubsystem subsystem;

    @Before
    public void setup() {
        assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
        Config config = new Config();

        // make the PID more aggressive for our tests since the simulated motor is a bit more idealized
        config.climber.climber1.positionPIDF = new PIDF(1, 0, 0, 0);
        config.climber.climber2.positionPIDF = new PIDF(1, 0, 0, 0);

        // create the subsystem without pneumatics
        config.enablePneumatics = false;
        subsystem = new ClimberSubsystem(config);
        subsystem.init();
    }

    @After
    public void tearDown() {

    }

    @Test
    public void testIdle() {

        // we should do nothing in periodic by default
        subsystem.periodic();

        // wait for the CTRE sim to update (weird)
        waitForCTREUpdate();

        // verify we applied no voltage to climb motors
        assertEquals(0, subsystem.climber1.getSimCollection().getMotorOutputLeadVoltage(), DELTA);
        assertEquals(0, subsystem.climber2.getSimCollection().getMotorOutputLeadVoltage(), DELTA);
    }

    @Test
    public void testAutoClimb() {
        // enable autoClimb
        subsystem.autoClimb();

        // we should start auto climb
        subsystem.periodic();

        // wait for the CTRE sim to update (weird)
        waitForCTREUpdate();

        // run the simuluation
        subsystem.simulationPeriodic();
        CTREPhysicsSim.getInstance().run();

        // we should be moving towards our goal
        assertNotEquals(0, subsystem.climber1.getSimCollection().getMotorOutputLeadVoltage());
        assertNotEquals(0, subsystem.climber2.getSimCollection().getMotorOutputLeadVoltage());
    }
}
