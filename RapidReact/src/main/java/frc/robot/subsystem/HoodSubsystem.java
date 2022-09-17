package frc.robot.subsystem;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.Config;
import frc.robot.utils.MotorUtils;

import java.util.Optional;

public class HoodSubsystem extends BitBucketsSubsystem {

    private CANSparkMax motor;
    
    private SparkMaxLimitSwitch m_forwardLimit;
    private SparkMaxLimitSwitch m_reverseLimit;

    private final LerpTable<Double, Double> angleTable = new LerpTable<>();


    static final String ACTUAL_REVOLUTIONS = "actual revs";
    private final VisionSubsystem visionSubsystem;

    final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0, 0, -9.8);


    //845/72 is number of motor turns to raise hood by 1 degree
    private final double HOOD_MOTOR_CONSTANT = 169f/9;


    public HoodSubsystem(Config config, VisionSubsystem visionSubsystem) {
        super(config);
        this.visionSubsystem = visionSubsystem;
    }


    @Override
    public void init() {
        motor = MotorUtils.makeSpark(config.hood.hoodMotor);

        SparkMaxPIDController pidController = motor.getPIDController();

        // configure position PID constants
        pidController.setFF(0);
        pidController.setP(0.03);
        pidController.setD(0 );
        pidController.setI(0);
        pidController.setIZone(0);
        pidController.setOutputRange(-1, 1);

        motor.getEncoder().setPosition(0);
        motor.setIdleMode(IdleMode.kBrake);
        
        this.m_forwardLimit = motor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        this.m_reverseLimit = motor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        m_forwardLimit.enableLimitSwitch(true);
        m_reverseLimit.enableLimitSwitch(true);




        //LERP Table Values
        //angleTable.put(distance, REVOLUTION THINGY)
        angleTable.put(0d,0d);
        angleTable.put(10d,18.9);
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
        SmartDashboard.putNumber(ACTUAL_REVOLUTIONS, motor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42).getPosition());
        SmartDashboard.putBoolean("Forward Limit Enabled", m_forwardLimit.isPressed());
        SmartDashboard.putBoolean("Reverse Limit Enabled", m_reverseLimit.isPressed());



    }


    @Override
    public void disable() {

    }

    public Optional<Double> getDesiredRevolutions() {
        if (visionSubsystem.hasTarget()) {
            return Optional.of(angleTable.get(visionSubsystem.distance()));
        }

        return Optional.empty();
    }

    public void hoodToRevolutions(double angle_revs) {
        motor.getPIDController().setReference(angle_revs, CANSparkMax.ControlType.kPosition);
    }

    public void hoodUp(){
        motor.set(0.1);;
    }

    public void hoodDown(){
        motor.set(-0.1);;
    }

    public void hoodStop(){
        motor.set(0);
    }



}
