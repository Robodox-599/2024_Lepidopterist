// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
    public static final int Ybutton = 0;//fix unused lol
    public static final int Xbutton = 1;//fix unused lol
  }
  public static class MotorID{
    public static final int climbMotorID = 0;//fix
    public static final String CANbusID = "rio";//fix
  }
  public static class PIDConstants{//fix
    public static final double climbP = 0.1;
    public static final double climbI = 0;
    public static final double climbD = 0;
    public static final double climbS = 0.05;
    public static final double climbV = 0.12;
    public static final double kinetic_friction = 0.25;
  }
  public static class Setpoints{//fix
    // public static final double start = 0;
    public static final double pull_up = 20;
    public static final double setPointDeadband = 2;
    public static final double increment1 = 100; //dist from base to top of elevator
    public static final double step = 5;

  }
}
