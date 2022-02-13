package frc.robot.subsystem;

public abstract class SubsystemTest {

    public static final double BUS_VOLTAGE = 12;

    protected void waitForCTREUpdate() {
        try {
            com.ctre.phoenix.unmanaged.Unmanaged.feedEnable(500);
            Thread.sleep(200);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

}
