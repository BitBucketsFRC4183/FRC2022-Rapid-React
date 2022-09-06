package frc.robot.lib.data.log;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.networktables.NetworkTableEntry;

import java.util.function.Consumer;

public class TalonEntry implements Consumer<WPI_TalonSRX> {

    final NetworkTableEntry voltageOutput;
    final NetworkTableEntry percentOutput;

    public TalonEntry(NetworkTableEntry voltageOutput, NetworkTableEntry percentOutput) {
        this.voltageOutput = voltageOutput;
        this.percentOutput = percentOutput;
    }

    @Override
    public void accept(WPI_TalonSRX wpi_talonSRX) {
        voltageOutput.setNumber(wpi_talonSRX.getMotorOutputVoltage());
        percentOutput.setNumber(wpi_talonSRX.getMotorOutputPercent());
    }
}
