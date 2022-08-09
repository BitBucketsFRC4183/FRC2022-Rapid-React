package frc.robot.subsystem.drive.cmd;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.vision.VisionSubsystem;
import frc.robot.subsystem.drive.DriveSubsystem2;

public class DriveOrient extends CommandBase {

    private final DriveSubsystem2 driveSubsystem;
    private final VisionSubsystem visionSubsystem;

    public DriveOrient(DriveSubsystem2 driveSubsystem, VisionSubsystem visionSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;

        this.addRequirements(driveSubsystem, visionSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }
}
