package frc.robot.intake;

import frc.robot.lib.data.Const;
import frc.robot.lib.data.Container;

import java.util.function.Supplier;

public interface BallManagement {

    Container BMS = Container.GLOBAL.sub("bms");

    Supplier<Double> BMS_FEED = BMS.constant(Const.PERCENT_NUMBER, "bms_feed_percent_output", 1.0);

}
