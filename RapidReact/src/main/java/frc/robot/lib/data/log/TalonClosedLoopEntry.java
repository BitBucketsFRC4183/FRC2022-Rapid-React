package frc.robot.lib.data.log;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.networktables.NetworkTableEntry;

import java.util.function.Consumer;

public class TalonClosedLoopEntry implements Consumer<WPI_TalonSRX> {

    final NetworkTableEntry voltageOutput;
    final NetworkTableEntry percentOutput;
    final NetworkTableEntry closedLoop;
    final NetworkTableEntry velocity;
    final NetworkTableEntry position;

    public TalonClosedLoopEntry(NetworkTableEntry voltageOutput, NetworkTableEntry percentOutput, NetworkTableEntry closedLoop, NetworkTableEntry velocity, NetworkTableEntry position) {
        this.voltageOutput = voltageOutput;
        this.percentOutput = percentOutput;
        this.closedLoop = closedLoop;
        this.velocity = velocity;
        this.position = position;
    }

    @Override
    public void accept(WPI_TalonSRX wpi_talonSRX) {
        position.setDoubleArray(new double[] { wpi_talonSRX.getActiveTrajectoryPosition(), wpi_talonSRX.getSelectedSensorPosition() } );
        closedLoop.setDoubleArray(new double[] { wpi_talonSRX.getClosedLoopTarget(), wpi_talonSRX.getClosedLoopError() });
        velocity.setDoubleArray(new double[] { wpi_talonSRX.getActiveTrajectoryVelocity(), wpi_talonSRX.getSelectedSensorVelocity() });

        voltageOutput.setNumber(wpi_talonSRX.getMotorOutputVoltage());
        percentOutput.setNumber(wpi_talonSRX.getMotorOutputPercent());
    }
}
