// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
 
  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class armConstants {
    public static final int motorArm = 30;
    public static final double kGearRatio = 5.0 * 3.0 * 32.0 / 18.0;
    public static final double kEncoderDegrees = 360.0 / kGearRatio;
    public static final double kArmUp = 85; // / kEncoderDegrees; // angle is negative degrees
    public static final double kArmDown = 170; // / kEncoderDegrees;
    public static final int kCurrentLimit = 20;
    public static final double kSecondaryCurrentLimit = 30;
    public static final double kPostionAdjust = 15; // / kEncoderDegrees; // angle is in degrees 
    public static final double kFwdSoftLimit = 180; // / kEncoderDegrees;
    public static final double kRevSoftLimit = 80; // / kEncoderDegrees;
    //public static final double kHandDownish = 105; // for absolute encoder actual degrees

 }


}
