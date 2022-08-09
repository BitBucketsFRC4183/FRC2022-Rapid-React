package frc.robot.lib.ex;

import frc.robot.Buttons;
import frc.robot.subsystem.RGBSubsystem;
import frc.robot.lib.Script;
import frc.robot.utils.BlinkenColors;

public class MonadScriptA implements Script {

    private final RGBSubsystem rgb; //script only active if rgb and xyz are on

    public MonadScriptA(RGBSubsystem rgb) {
        this.rgb = rgb;
    }

    //commands
    public void turnRed() {
        rgb.setColor(BlinkenColors.Colors_Red);
    }

    public void turnOrange() {
        rgb.setColor(BlinkenColors.Colors_Orange);
    }

    //stateful bindings
    @Override
    public void buttons(Buttons buttons) {
        buttons.elevatorRetract.whenPressed(this::turnOrange);
        buttons.elevatorRetract.whenReleased(this::turnRed);
    }

}
