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

  public static class CANIdConstants {
    public static final int ID_MOTEUR_GAUCHE = 1;
    public static final int ID_MOTEUR_GAUCHE_FOLLOWER = 10;
    public static final int ID_MOTEUR_DROIT = 3;
    public static final int ID_MOTEUR_DROIT_FOLLOWER = 4;

    public static final int ID_MOTEUR_ELEVATOR = 5;
    public static final int ID_MOTEUR_INTAKE_ALGUE = 6;
    public static final int ID_MOTEUR_BRAS = 7;
    public static final int ID_MOTEUR_OUTTAKE_ALGUE = 2;
    public static final int ID_MOTEUR_OUTTAKE_CORAL = 9;
    

  }
  public static class DriveConstants {

    public static final boolean kRightInverted = true;
    public static final boolean kLeftInverted = false;
     
    public static final double kvVoltSecondsPerMeter = 1.4977;
    public static final double ksVolt = 11.332;
    public static final double kaVoltSecondsSquaredPerMeter = 7.3102;
    public static final double kp = 0.17843;
    

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

 

  
  public static final class ElevatorConstants {

    public static final double ELEVATOR_DOWN_POSITION = 0.2; // a définir
    public static final double ELEVATOR_L2_POSITION = 28.14; // a définir
    public static final double ELEVATOR_L3_POSITION = 65;
    public static final double ELEVATOR_L1_POSITION = 14.7; // a définir
    public static final double kp = 0.06;
    public static final double kg = 0.03;
  }

  public static final class ArmConstants {

    public static final double ENCODER_DEGREES = (1/152)*360;
    
  }
}
