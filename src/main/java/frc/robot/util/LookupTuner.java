package frc.robot.util;

import static frc.robot.Constants.ShooterConstants.shooterLUT;

public class LookupTuner {
  private static LoggedTunableNumber[][] lookupTable =
      new LoggedTunableNumber[shooterLUT.length][shooterLUT[0].length];

  public static void setupTuner() {
    for (int i = 0; i < shooterLUT.length; i++) {
      for (int j = 1; j < shooterLUT[i].length; j++) {
        System.out.print(shooterLUT[i][j] + " ");
        String text = j == 1 ? "RPM" : "Angle";
        lookupTable[i][j] =
            new LoggedTunableNumber(
                "shooterLUT/" + "Meters-" + shooterLUT[i][0] + "/" + text, shooterLUT[i][j]);
      }
    }
  }

  public static double getMatrixValue(int i, int j) {
    return lookupTable[i][j].get();
  }

  public static void updateMatrix() {
    for (int i = 0; i < shooterLUT.length; i++) {
      for (int j = 1; j < shooterLUT[i].length; j++) {
        if (lookupTable[i][j].get() != shooterLUT[i][j]) {
          shooterLUT[i][j] = lookupTable[i][j].get();
        }
      }
    }
  }
}
