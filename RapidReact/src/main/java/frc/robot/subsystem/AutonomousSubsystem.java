package frc.robot.subsystem;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.Config;

public class AutonomousSubsystem extends BitBucketsSubsystem {


    public Field2d field;

    public AutonomousSubsystem(Config config) {
        super(config);

    }

    public void setTrajectory(Trajectory trajectory) {
        field.getObject("traj").setTrajectory(trajectory);
    }

    @Override
    public void init() {
    }

    @Override
    public void periodic() {

    }

    @Override
    public void disable() {

    }

}
