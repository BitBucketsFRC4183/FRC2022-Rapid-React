package frc.robot.subsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxLimitSwitch;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.Config;
import frc.robot.utils.MotorUtils;

public class HoodSubsystem extends BitBucketsSubsystem {

    private double hood_angle = 0;
    private CANSparkMax hoodMotor;
    private SparkMaxLimitSwitch m_forwardLimit;
    private SparkMaxLimitSwitch m_reverseLimit;

    private final LerpTable<Double, Double> angleTable = new LerpTable<>();
    private final LerpTable<Double, Double> lowMotorTable = new LerpTable<>();
    private final LerpTable<Double, Double> highMotorTable = new LerpTable<>();

    private final VisionSubsystem visionSubsystem;

    final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0, 0, -9.8);


    //845/72 is number of motor turns to raise hood by 1 degree
    private final double HOOD_MOTOR_CONSTANT = 845f/72;

    //TODO
    private final LerpTable<Double, Double> shooterBottom = new LerpTable<>();
    private final LerpTable<Double, Double> shooterTop = new LerpTable<>();


    public HoodSubsystem(Config config, VisionSubsystem visionSubsystem) {
        super(config);
        this.visionSubsystem = visionSubsystem;
    }


    @Override
    public void init() {
        SmartDashboard.putNumber("hood-p", 0.0);
        SmartDashboard.putNumber("hood-i", 0.0);
        SmartDashboard.putNumber("hood-d", 0.0);

        hoodMotor = MotorUtils.makeSpark(config.hood.hoodMotor);
        hoodMotor.setIdleMode(IdleMode.kBrake);

        ShuffleboardContainer tab = Shuffleboard.getTab("HoodTesting");

        SmartDashboard.putNumber("Hood Angle setpoint", hood_angle);
        SmartDashboard.putNumber("Hood angle current poss", hoodMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Output percent",hoodMotor.getAppliedOutput());
        SmartDashboard.putNumber("Hood Error",hood_angle-hoodMotor.getEncoder().getPosition());

        //log shit

        
        this.m_forwardLimit = hoodMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        this.m_reverseLimit = hoodMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        m_forwardLimit.enableLimitSwitch(true);
        m_reverseLimit.enableLimitSwitch(true);




        //LERP Table Values
        //angleTable.put(distance, angle)
        angleTable.put(0d,0d);
        angleTable.put(100d,100d);

        //angleTable.put(distance, speed)
        lowMotorTable.put(0d,0d);
        lowMotorTable.put(100d,100d);

        //angleTable.put(distance, speed)
        highMotorTable.put(0d,0d);
        highMotorTable.put(100d,100d);


    }

    @Override
    public void periodic() {

//        m_forwardLimit.enableLimitSwitch(SmartDashboard.getBoolean("Forward Limit Enabled", false));
//        m_reverseLimit.enableLimitSwitch(SmartDashboard.getBoolean("Reverse Limit Enabled", false));
//        SmartDashboard.putBoolean("Forward Limit Switch", m_forwardLimit.isPressed());
//        SmartDashboard.putBoolean("Reverse Limit Switch", m_reverseLimit.isPressed());
//
//        if(m_forwardLimit.isLimitSwitchEnabled()){
//            m_forwardLimit.enableLimitSwitch(true);
//        }
//        if(m_reverseLimit.isLimitSwitchEnabled()){
//            m_reverseLimit.enableLimitSwitch(true);
//        }


        hood_angle = SmartDashboard.getNumber("Hood Angle setpoint", 0.0);
        SmartDashboard.putNumber("Hood Angle setpoint", hood_angle);
        SmartDashboard.putNumber("Hood angle current poss", hoodMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Output percent",hoodMotor.getAppliedOutput());
        SmartDashboard.putNumber("Hood Error",hood_angle-hoodMotor.getEncoder().getPosition());
        SmartDashboard.putBoolean("Forward Limit Enabled", m_forwardLimit.isPressed());
        SmartDashboard.putBoolean("Reverse Limit Enabled", m_reverseLimit.isPressed());

        if (m_reverseLimit.isPressed())
        {
            hoodMotor.getEncoder().setPosition(0);
        }

        //set stuff

        double p = SmartDashboard.getNumber("hood-p", 0.0);
        double i = SmartDashboard.getNumber("hood-i", 0.0);
        double d = SmartDashboard.getNumber("hood-d", 0.0);

        hoodMotor.getPIDController().setP(p);
        hoodMotor.getPIDController().setI(i);
        hoodMotor.getPIDController().setD(d);

        setAngle(hood_angle);

    }

    @Override
    public void disable() {

    }


    public void setAngle(double angle_degrees) {
        //TODO

        //sets number of rotations of the motor to move the hood by a certain angle parameter
        hoodMotor.getPIDController().setReference(HOOD_MOTOR_CONSTANT * angle_degrees, CANSparkMax.ControlType.kPosition);
    
    }

    public void hoodUp(){
        //hoodMotor.set(0.1);;
    }


    public void hoodDown(){
        //hoodMotor.set(-0.1);;
    }

    public void hoodStop(){
        hoodMotor.set(0);
    }

    public double getTopShootSpeed() {
        return highMotorTable.get(visionSubsystem.distance());
    }

    public double getBottomShootSpeed() {
        return lowMotorTable.get(visionSubsystem.distance());
    }

}
