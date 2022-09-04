package frc.robot.subsystem;

import com.revrobotics.CANSparkMax;
import frc.robot.config.Config;

public class HoodSubsystem extends BitBucketsSubsystem {

    private CANSparkMax motor;

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
    }

    @Override
    public void periodic() {
        if(visionSubsystem.hasTarget()){
            double angle = angleTable.get(visionSubsystem.distance());
            setAngle(angle);
        }
    }

    @Override
    public void disable() {

    }


    void setAngle(double angle_degrees) {
        //TODO
        //sets number of rotations of the motor to move the hood by a certain angle parameter
        motor.getPIDController().setReference(HOOD_MOTOR_CONSTANT * angle_degrees, CANSparkMax.ControlType.kPosition);
    }
}
