package frc.robot.lib.exec;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ExecutableWrapper extends CommandBase {

    private final Executable executable;

    public ExecutableWrapper(Executable executable) {
        this.executable = executable;
    }

    @Override
    public void execute() {
        executable.run(Status.NORMAL);
    }

    @Override
    public void initialize() {
        executable.run(Status.INIT);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            executable.run(Status.SHUTDOWN_INTERRUPT);
        } else {
            executable.run(Status.SHUTDOWN);
        }
    }
}
