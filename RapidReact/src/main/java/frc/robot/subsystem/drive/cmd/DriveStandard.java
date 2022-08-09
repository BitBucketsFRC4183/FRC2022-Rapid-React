package frc.robot.subsystem.drive.cmd;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.drive.DriveSubsystem2;

public class DriveStandard extends CommandBase {

    private final Joystick joystick = new Joystick(1);
    private final DriveSubsystem2 driveSubsystem;

    public DriveStandard(DriveSubsystem2 driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }
}
