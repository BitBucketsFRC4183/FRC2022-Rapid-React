package frc.robot;

import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.utils.PS4Constants;

//i am thieving code
public class CursedRobot extends TimedRobot {

  /*
   * --- [1] Update CAN Device IDs ------
   */
  WPI_TalonFX _rghtFront = new WPI_TalonFX(1, "rio");
  WPI_TalonFX _rghtFollower = new WPI_TalonFX(4, "rio");
  WPI_TalonFX _leftFront = new WPI_TalonFX(2, "rio");
  WPI_TalonFX _leftFollower = new WPI_TalonFX(3, "rio");

  DifferentialDrive _diffDrive = new DifferentialDrive(_leftFront, _rghtFront);

  Joystick _joystick = new Joystick(0);

  Faults _faults_L = new Faults();
  Faults _faults_R = new Faults();

  float robotSpeedMultiplier = 1f;

  @Override
  public void teleopPeriodic() {
    String work = "";

    /* get gamepad stick values *///should be ps4
    double forw = -_joystick.getRawAxis(
      PS4Constants.LEFT_STICK_Y.getId()
    );/* positive is forward */
    double turn = -_joystick.getRawAxis(
      PS4Constants.RIGHT_STICK_X.getId()
    );/* positive is right */

    /* deadband gamepad 10% */
    if (Math.abs(forw) < 0.10) {
      forw = 0;
    }
    if (Math.abs(turn) < 0.10) {
      turn = 0;
    }

    double leftSpeed = (forw + turn * -1);
    double rightSpeed = (forw - turn * -1);

    /* drive robot */
    _diffDrive.tankDrive(
      leftSpeed * robotSpeedMultiplier,
      rightSpeed * robotSpeedMultiplier
    );

    /*
     * [2] Make sure Gamepad Forward is positive for FORWARD, and GZ is positive for
     * RIGHT
     */
    work += " GF:" + forw + " GT:" + turn;

    /* get sensor values */
    // double leftPos = _leftFront.GetSelectedSensorPosition(0);
    // double rghtPos = _rghtFront.GetSelectedSensorPosition(0);
    double leftVelUnitsPer100ms = _leftFront.getSelectedSensorVelocity(0);
    double rghtVelUnitsPer100ms = _rghtFront.getSelectedSensorVelocity(0);

    work += " L:" + leftVelUnitsPer100ms + " R:" + rghtVelUnitsPer100ms;

    /*
     * drive motor at least 25%, Talons will auto-detect if sensor is out of phase
     */
    _leftFront.getFaults(_faults_L);
    _rghtFront.getFaults(_faults_R);

    if (_faults_L.SensorOutOfPhase) {
      work += " L sensor is out of phase";
    }
    if (_faults_R.SensorOutOfPhase) {
      work += " R sensor is out of phase";
    }
    // System.out.println(work);
  }

  @Override
  public void robotInit() {
    /* factory default values */
    _rghtFront.configFactoryDefault();
    _rghtFollower.configFactoryDefault();
    _leftFront.configFactoryDefault();
    _leftFollower.configFactoryDefault();

    /* set up followers */
    _rghtFollower.follow(_rghtFront);
    _leftFollower.follow(_leftFront);

    /* [3] flip values so robot moves forward when stick-forward/LEDs-green */
    _rghtFront.setInverted(TalonFXInvertType.Clockwise); // !< Update this
    _leftFront.setInverted(TalonFXInvertType.CounterClockwise); // !< Update this

    /*
     * set the invert of the followers to match their respective master controllers
     */
    _rghtFollower.setInverted(InvertType.FollowMaster);
    _leftFollower.setInverted(InvertType.FollowMaster);
    /*
     * Talon FX does not need sensor phase set for its integrated sensor
     * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
     * and the user calls getSelectedSensor* to get the sensor's position/velocity.
     *
     * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
     */
    // _rghtFront.setSensorPhase(true);
    // _leftFront.setSensorPhase(true);
  }
}
