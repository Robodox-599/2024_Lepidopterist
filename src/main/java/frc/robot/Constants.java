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
}
public static class MotorID{
public static final int motorID_Left = 0;//fix
public static final int motorID_Right = 6;//fix
public static final int gyroID = 12;

public static final String CANbusID = "DriveCANivore";//fix
}
public static class PIDConstants{//fix
public static final double extendP = 0.1;
public static final double extendI = 0;
public static final double extendD = 0.01;

public static final double balanceP = 0.01;
public static final double balanceI = 0;
public static final double balanceD = 0.01;

}
public static class Setpoints{//fix
// public static final double start = 0;
public static final double stow = 0;
public static final double extended = 100;
public static final double retracted = 50;
public static final double setPointDeadband = 3;
public static final double balanceLimit = 20;

}

public static class GyroConstants{
    public static final double gyroThreshold = 1.5; // TODO: degrees?? idk
    public static final double errorGain = 0.05;

}

}



