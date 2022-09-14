package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.Config;
import frc.robot.config.MotorConfig;
import frc.robot.utils.MotorUtils;

public class HoodSubsystem extends BitBucketsSubsystem {

    private double hood_angle = 0;
    private CANSparkMax hoodMotor;
//    private SparkMaxLimitSwitch m_forwardLimit;
//    private SparkMaxLimitSwitch m_reverseLimit;

    private final LerpTable<Double, Double> angleTable = new LerpTable<>();

    private final VisionSubsystem visionSubsystem;

    //845/72 is number of motor turns to raise hood by 1 degree
    private final double HOOD_MOTOR_CONSTANT = 845f/72;


    public HoodSubsystem(Config config, VisionSubsystem visionSubsystem) {
        super(config);
        this.visionSubsystem = visionSubsystem;
    }


    @Override
    public void init() {
        hoodMotor = MotorUtils.makeSpark(config.hood.hoodMotor);
        hoodMotor.setIdleMode(IdleMode.kBrake);

        
//        hoodMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
//        hoodMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
//        m_forwardLimit.enableLimitSwitch(false);
//        m_reverseLimit.enableLimitSwitch(false);
//        SmartDashboard.putBoolean("Forward Limit Enabled", m_forwardLimit.isLimitSwitchEnabled());
//        SmartDashboard.putBoolean("Reverse Limit Enabled", m_reverseLimit.isLimitSwitchEnabled());
//        m_forwardLimit.enableLimitSwitch(SmartDashboard.getBoolean("Forward Limit Enabled", false));
//        m_reverseLimit.enableLimitSwitch(SmartDashboard.getBoolean("Reverse Limit Enabled", false));

        //LERP Table Values
        //angleTable.put(distance, angle)
        angleTable.put(0d,0d);
        angleTable.put(100d,100d);
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
        if(visionSubsystem.hasTarget()){
            double angle = angleTable.get(visionSubsystem.distance());
            hood_angle = angle;
            setAngle(angle);

        }
        SmartDashboard.putNumber("Hood Angle setpoint", hood_angle);
        SmartDashboard.putNumber("Hood angle current poss", hoodMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Output percent",hoodMotor.getAppliedOutput());
        SmartDashboard.putNumber("Hood Error",hood_angle-hoodMotor.getEncoder().getPosition());

    }

    @Override
    public void disable() {

    }


    void setAngle(double angle_degrees) {
        //TODO

        //sets number of rotations of the motor to move the hood by a certain angle parameter
        hoodMotor.getPIDController().setReference(HOOD_MOTOR_CONSTANT * angle_degrees, CANSparkMax.ControlType.kPosition);
    
    }

    public void hoodUp(){
        hoodMotor.set(0.1);;
    }

    public void hoodDown(){
        hoodMotor.set(-0.1);;
    }

    public void hoodStop(){
        hoodMotor.set(0);
    }
}
