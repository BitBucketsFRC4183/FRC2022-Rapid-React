package frc.robot.utils;

public enum BlinkenColors {
  RainbowRainbowPalette(-0.99),
  RainbowPartyPalette(-0.97),
  RainbowOceanPalette(-0.95),
  RainbowLavePalette(-0.93),
  RainbowForestPalette(-0.91),
  RainbowwithGlitter(-0.89),
  Confetti(-0.87),
  ShotRed(-0.85),
  ShotBlue(-0.83),
  ShotWhite(-0.81),
  SinelonRainbowPalette(-0.79),
  SinelonPartyPalette(-0.77),
  SinelonOceanPalette(-0.75),
  SinelonLavaPalette(-0.73),
  SinelonForestPalette(-0.71),
  BeatsperMinuteRainbowPalette(-0.69),
  BeatsperMinutePartyPalette(-0.67),
  BeatsperMinuteOceanPalette(-0.65),
  BeatsperMinuteLavaPalette(-0.63),
  BeatsperMinuteForestPalette(-0.61),
  FireMedium(-0.59),
  FireLarge(-0.57),
  TwinklesRainbowPalette(-0.55),
  TwinklesPartyPalette(-0.53),
  TwinklesOceanPalette(-0.51),
  TwinklesLavaPalette(-0.49),
  TwinklesForestPalette(-0.47),
  ColorWavesRainbowPalette(-0.45),
  ColorWavesPartyPalette(-0.43),
  ColorWavesOceanPalette(-0.41),
  ColorWavesLavaPalette(-0.39),
  ColorWavesForestPalette(-0.37),
  LarsonScannerRed(-0.35),
  LarsonScannerGray(-0.33),
  LightChaseRed(-0.31),
  LightChaseBlue(-0.29),
  LightChaseGray(-0.27),
  HeartbeatRed(-0.25),
  HeartbeatBlue(-0.23),
  HeartbeatWhite(-0.21),
  HeartbeatGray(-0.19),
  BreathRed(-0.17),
  BreathBlue(-0.15),
  BreathGray(-0.13),
  StrobeRed(-0.11),
  StrobeBlue(-0.09),
  StrobeGold(-0.07),
  StrobeWhite(-0.05),
  EndtoEndBlendtoBlack(-0.03),
  LarsonScanner(-0.01),
  LightChase(0.01),
  HeartbeatSlow(0.03),
  HeartbeatMedium(0.05),
  HeartbeatFast(0.07),
  BreathSlow(0.09);

  private double value;

  private BlinkenColors(double id) {
    this.value = id;
  }

  public double getValue() {
    return value;
  }
}
