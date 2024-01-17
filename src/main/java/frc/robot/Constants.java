

package frc.robot;


/**
* The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
* constants. This class should not be used for any other purpose. All constants should be declared
* globally (i.e. public static). Do not put anything functional in this class.
*
* <p>It is advised to statically import this class (or one of its inner classes) wherever the
* constants are needed, to reduce verbosity.
*/
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class MotorConstants{
    public static final int flywheel1ID = 0;
    public static final int flywheel2ID = 0;
    public static final int feederID = 0;
    public static final int wristID = 0;
    public static final String canbusID = "rio";//uhh idk for this one
    public static final int DIO_pin1 = 0;
    public static final int DIO_pin2 = 0;
    public static final double feeder_voltage = 8;
  }

  public static class PIDConstants{
    public static final double flywheelkP = 0.0;
    public static final double flywheelkI = 0.0;
    public static final double flywheelkD = 0.0;
    public static final double flywheelkS = 0.0;
    public static final double flywheelkV = 0.0;
    public static final double wristkP = 0.0;
    public static final double wristkI = 0.0;
    public static final double wristkD = 0.0;
  }

  public static class ShooterWristConstants{
    public static final double ampWrist = 12.0; //change
    public static final double minWrist = 0.0; //change
    public static final double maxWrist = 12.0; //change

  }

}


