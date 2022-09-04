package frc.robot.lib.exec;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ExecutableWrapper extends CommandBase {

    private final Executable executable;

    public ExecutableWrapper(Executable executable) {
        this.executable = executable;
    }

    @Override
    public void execute() {
        executable.run(CommandIs.RUNNING);
    }

    @Override
    public void initialize() {
        executable.run(CommandIs.STARTING);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            executable.run(CommandIs.STOPPING_INTERRUPTED);
        } else {
            executable.run(CommandIs.STOPPING);
        }
    }
}
