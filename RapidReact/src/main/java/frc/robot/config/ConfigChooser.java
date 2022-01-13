package frc.robot.config;

import frc.robot.config.urations.JuniorConfig;

public class ConfigChooser {
  public static enum Bot {
    Nameless, Junior
  };

  // Change this line to switch configurations.
  static Bot currentRobot = Bot.Nameless;

  public static Config GetConfig() {
    switch (currentRobot) {
    case Nameless:
      return new Config();
    case Junior:
      return new JuniorConfig();
    default:
      return new Config();
    }
  }

}
