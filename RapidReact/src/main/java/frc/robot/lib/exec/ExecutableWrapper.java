package frc.robot.lib.exec;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ExecutableWrapper extends CommandBase {

    private final Executable executable;

    public ExecutableWrapper(Executable executable) {
        this.executable = executable;
    }

    @Override
    public void execute() {
        executable.run(CommandState.NORMAL);
    }

    @Override
    public void initialize() {
        executable.run(CommandState.INIT);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            executable.run(CommandState.SHUTDOWN_INTERRUPT);
        } else {
            executable.run(CommandState.SHUTDOWN);
        }
    }
}
