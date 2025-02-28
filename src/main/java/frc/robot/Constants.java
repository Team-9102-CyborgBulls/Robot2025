// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

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
  public static class DriveConstants {

    public static final boolean kRightInverted = true;
    public static final boolean kLeftInverted = false;
     
    public static final double kvVoltSecondsPerMeter = 1.4977;
    public static final double ksVolt = 11.332;
    public static final double kaVoltSecondsSquaredPerMeter = 7.3102;
    public static final double kp = 0.17843;

    public static final double MAX_DRIVING_VELOCITY_METERS_PER_SECOND = 5;
        public static final double MAX_DRIVING_ACCELERATION_METERS_PER_SECOND_SQUARED = 8;
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 20;
        public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 40;
      
    

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final double kTrackwidthMeters = 0.58;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final double kMaxSpeedMetersPerSecond = 3.41;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1.84;

    public static  int kEncoderCPR = 2048;
    public static double kWheelDiameterMeters = Units.inchesToMeters(6);
    public static double kEncoderDistancePerPulse = (kWheelDiameterMeters * Math.PI) / kEncoderCPR;

  }

  public static final class Controls {
    public static final double JOYSTICK_INPUT_RATE_LIMIT = 15.0;
    public static final double JOYSTICK_INPUT_DEADBAND = 0.1;
    public static final double JOYSTICK_CURVE_EXP = 2;
    public static final double JOYSTICK_ROT_CURVE_EXP = 3;
    public static final double JOYSTICK_ROT_LIMIT = 0.8;
}
  
  public static final class ElevatorConstants {

    public static final double ELEVATOR_DOWN_POSITION = 0.2; // a définir
    public static final double ELEVATOR_INTAKE_POSITION = 0.2; // a définir
    public static final double ELEVATOR_L2_POSITION = 0.2; // a définir
    public static final double ELEVATOR_L3_POSITION = 0.2; // a définir
  }
}
