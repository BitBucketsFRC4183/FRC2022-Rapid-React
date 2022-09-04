package frc.robot.lib.data.log;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.networktables.NetworkTableEntry;

import java.util.function.Consumer;

public class TalonClosedLoopEntry implements Consumer<WPI_TalonSRX> {

    final NetworkTableEntry error;
    final NetworkTableEntry


    @Override
    public void accept(WPI_TalonSRX wpi_talonSRX) {
        wpi_talonSRX.getClosedLoopError();


    }
}
