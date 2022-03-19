package frc.robot.commands;

import static org.junit.Assert.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import frc.robot.commands.DriveAtRotation;
import frc.robot.config.Config;
import frc.robot.subsystem.DrivetrainSubsystem;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

public class DriveAtRotationTest {

  public static final double DELTA = 0.1; // acceptable deviation range

  DrivetrainSubsystem subsystem;
  Config config = new Config();

  @Before
  public void setup() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed

    // create the subsystem
    subsystem = new DrivetrainSubsystem(config);
    subsystem.init();
  }

  @Test
  public void testGetSnapTpAngle() {
    var command = new DriveAtRotation(subsystem, () -> 0, () -> 0, config);
    assertEquals((-22), Math.toDegrees(command.getSnapToAngle(0)), DELTA);
    assertEquals((158), Math.toDegrees(command.getSnapToAngle(180)), DELTA);
    assertEquals((158), Math.toDegrees(command.getSnapToAngle(-185)), DELTA);
    assertEquals((68), Math.toDegrees(command.getSnapToAngle(40)), DELTA);
    assertEquals((-112), Math.toDegrees(command.getSnapToAngle(-100)), DELTA);
    assertEquals((158), Math.toDegrees(command.getSnapToAngle(190)), DELTA);
  }
}
