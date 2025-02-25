// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.pathplanner.lib.config.RobotConfig;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.wpilibj.RobotController;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.math.controller.DifferentialDriveFeedforward;
import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;

import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.Encoder;


import com.revrobotics.jni.CANSparkJNI;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
//import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants;


 
public class DriveSubsystem extends SubsystemBase {
  
     
     private DifferentialDriveOdometry m_odometry;
     public double direction = 1.0;
     public double speed_changer = 0.6;

    SparkMax m_MotorRight = new SparkMax(4, MotorType.kBrushed);
    SparkMax m_MotorRightFollow = new SparkMax(3, MotorType.kBrushed);
    SparkMax m_MotorLeft = new SparkMax(1, MotorType.kBrushed);
    SparkMax m_MotorLeftFollow = new SparkMax(2, MotorType.kBrushed);

    static SparkMaxConfig config = new SparkMaxConfig();
    public static AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

    
    private final DifferentialDrive m_drive =
    new DifferentialDrive(m_MotorRight::set, m_MotorLeft::set);

    public static Encoder encoderDriveR = new Encoder(0,1, false);
    public static Encoder encoderDriveL = new Encoder(2,3,true);
             
    

    

      // Odometry class for tracking robot pose
    
                    
    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutVoltage m_appliedVoltage = Volts.mutable(0);
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutDistance m_distance = Meters.mutable(0);
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);
                    
    // Create a new SysId routine for characterizing the drive.
    private final SysIdRoutine m_sysIdRoutine =
        new SysIdRoutine(
    // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
            new SysIdRoutine.Config(Volts.of(1).per(Seconds),Volts.of(7),null,null),
            new SysIdRoutine.Mechanism(
    // Tell SysId how to plumb the driving voltage to the motors.
            voltage -> {
                m_MotorRight.setVoltage(voltage);
                m_MotorLeft.setVoltage(voltage);
                },
    // Tell SysId how to record a frame of data for each motor on the mechanism being
    // characterized.
        log -> {
    // Record a frame for the left motors.  Since these share an encoder, we consider
    // the entire group to be one motor.
        log.motor("drive-left")
            .voltage(
                m_appliedVoltage.mut_replace(
                m_MotorLeft.get() * RobotController.getBatteryVoltage(), Volts))
                  .linearPosition(m_distance.mut_replace(encoderDriveL.getDistance(), Meters))
                  .linearVelocity(m_velocity.mut_replace(encoderDriveL.getRate(),MetersPerSecond));
// Record a frame for the right motors.  Since these share an encoder, we consider
// the entire group to be one motor.
        log.motor("drive-right")
            .voltage(
                m_appliedVoltage.mut_replace(
                m_MotorRight.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(m_distance.mut_replace(encoderDriveR.getDistance(), Meters))
                    .linearVelocity(m_velocity.mut_replace(encoderDriveR.getRate(), MetersPerSecond));
        },
// Tell SysId to make generated commands require this subsystem, suffix test state in
// WPILog with this subsystem's name ("drive")
  this));
                                            
                                                 
                                                  
public DriveSubsystem() {
                                              
    
  config.voltageCompensation(12);
  config.smartCurrentLimit(40);
  config.idleMode(IdleMode.kCoast);

  config.follow(m_MotorRight);
  m_MotorRightFollow.configure(config,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);

  config.follow(m_MotorLeft);

  m_MotorLeftFollow.configure(config,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);

  config.disableFollowerMode();
  m_MotorRight.configure(config,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);

  config.inverted(true);
  m_MotorLeft.configure(config,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
                                            
  SendableRegistry.addChild(m_drive, m_MotorLeft);
  SendableRegistry.addChild(m_drive, m_MotorRight);


  encoderDriveL.setDistancePerPulse(Constants.DriveConstants.kEncoderDistancePerPulse);
  encoderDriveR.setDistancePerPulse(Constants.DriveConstants.kEncoderDistancePerPulse);
  encoderDriveL.reset();
  encoderDriveR.reset();
  
  m_odometry =
        new DifferentialDriveOdometry(
            gyro.getRotation2d(), encoderDriveL.getDistance(), encoderDriveR.getDistance());

   RobotConfig config;
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    /*AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPLTVController(0.02), // PPLTVController is the built in path following controller for differential drive trains
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );*/
                                                                                          
}
                                              
// SysID methods :

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
      return m_sysIdRoutine.dynamic(direction);
  }

//Drive methods :


private void setVoltage(double volts) {
  m_MotorLeft.setVoltage(volts);
  m_MotorRight.setVoltage(volts);
}

//@Override
public void drive(ChassisSpeeds speeds) {
  DifferentialDriveWheelSpeeds wheelSpeeds = Constants.DriveConstants.kDriveKinematics.toWheelSpeeds(speeds);
  wheelSpeeds.desaturate(Constants.DriveConstants.MAX_DRIVING_VELOCITY_METERS_PER_SECOND);

  m_MotorLeft.setVoltage((wheelSpeeds.leftMetersPerSecond / Constants.DriveConstants.MAX_DRIVING_VELOCITY_METERS_PER_SECOND) * 12.0);
  m_MotorRight.setVoltage((wheelSpeeds.rightMetersPerSecond / Constants.DriveConstants.MAX_DRIVING_VELOCITY_METERS_PER_SECOND) * 12.0);
}

public Command teleopDriveCommand(DoubleSupplier leftStickThrustSupplier,
            DoubleSupplier rightStickRotationSupplier) {
            
        SlewRateLimiter leftThrustLimiter = new SlewRateLimiter(Constants.Controls.JOYSTICK_INPUT_RATE_LIMIT);
        SlewRateLimiter rightThrustLimiter = new SlewRateLimiter(Constants.Controls.JOYSTICK_INPUT_RATE_LIMIT);
        SlewRateLimiter rotationLimiter = new SlewRateLimiter(Constants.Controls.JOYSTICK_INPUT_RATE_LIMIT);

        return run(() -> {
            double leftThrust = leftStickThrustSupplier.getAsDouble();
            //double rightThrust = rightStickThrustSupplier.getAsDouble();
            double rightRotation = rightStickRotationSupplier.getAsDouble();
            rightRotation *= Constants.Controls.JOYSTICK_ROT_LIMIT;
            

            leftThrust = Math.copySign(Math.pow(leftThrust, Constants.Controls.JOYSTICK_CURVE_EXP), leftThrust);
            //rightThrust = Math.copySign(Math.pow(rightThrust, Constants.Controls.JOYSTICK_CURVE_EXP), rightThrust);
            rightRotation = Math.copySign(Math.pow(rightRotation, Constants.Controls.JOYSTICK_ROT_CURVE_EXP), rightRotation);

            leftThrust = leftThrustLimiter.calculate(leftThrust);
            //rightThrust = rightThrustLimiter.calculate(rightThrust);
            rightRotation = rotationLimiter.calculate(rightRotation);

            
                
                    leftThrust *= Constants.DriveConstants.MAX_DRIVING_VELOCITY_METERS_PER_SECOND;
                    //rightThrust *= Constants.DriveConstants.MAX_DRIVING_VELOCITY_METERS_PER_SECOND;
                    rightRotation *= Constants.DriveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

                    drive(new ChassisSpeeds(leftThrust, 0, rightRotation));
                
        }).withName("drivetrain.teleopDrive");
      }




  public void arcadeDrive(double fwd, double rot){
    
    /*double previousSpeed = 0.0;
    final double rampRate = 0.05; // Limite de variation par boucle (ajustable)

    double targetSpeed = fwd;
     // Appliquer la rampe d'accélération
     if (targetSpeed > previousSpeed + rampRate) {
      targetSpeed = previousSpeed + rampRate;
  } else if (targetSpeed < previousSpeed - rampRate) {
      targetSpeed = previousSpeed - rampRate;
  }
  previousSpeed = targetSpeed; // Mise à jour pour la prochaine itération*/

  
    m_drive.arcadeDrive(fwd*direction, rot);
    //m_drive.setMaxOutput(speed_changer);
  }

  //public double getTargetSpeed(double fwd, double previousSpeed){

    
    /*final double rampRate = 0.05; // Limite de variation par boucle (ajustable)

    double targetSpeed = fwd;
     // Appliquer la rampe d'accélération
     if (targetSpeed > previousSpeed + rampRate) {
      targetSpeed = previousSpeed + rampRate;
  } else if (targetSpeed < previousSpeed - rampRate) {
      targetSpeed = previousSpeed - rampRate;
  }
  previousSpeed = targetSpeed; // Mise à jour pour la prochaine itération*/
  //return targetSpeed;
  //}*/
  /*
  public Command arcadeDriveCommand(DoubleSupplier fwd, DoubleSupplier rot) {
    
    return run(() -> m_drive.arcadeDrive(-fwd.getAsDouble(), rot.getAsDouble()))
        .withName("arcadeDrive");
  }
  */
  public void tankDrive(double left, double right){
    m_drive.tankDrive(left, right);
  }
  
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_MotorLeft.setVoltage(leftVolts);
    m_MotorRight.setVoltage(rightVolts);
    m_drive.feed();
  }

  public void setDriveMotors(double forward, double turn){

    double left = forward - turn;
    double right = forward + turn;

    m_MotorRight.set(right);
    m_MotorLeft.set(left);

    m_MotorRightFollow.set(right);
    m_MotorLeftFollow.set(left);
  }
  public void stop(){
      m_MotorRight.set(0.0);
      m_MotorRightFollow.set(0.0);
      m_MotorLeft.set(0.0);
      m_MotorLeftFollow.set(0.0);
  }
  
  public void drive(double leftPercentPower, double rightPercentPower) {
    m_MotorLeft.set(leftPercentPower);
    m_MotorLeftFollow.set(leftPercentPower);
    m_MotorRight.set(rightPercentPower);
    m_MotorRightFollow.set(rightPercentPower);
  }



  public void driveRight(double speed){
    m_MotorRight.set(speed);

  }
  public void driveLeft(double speed){
    m_MotorLeft.set(speed);
    
  }
  public void driveRightFollow(double speed){
    m_MotorRightFollow.set(speed);
    
  }
  public void driveLeftFollow(double speed){
    m_MotorLeftFollow.set(speed);
    

  }
  

  
  /*public void setFollowers() {
    m_MotorLeftFollow.follow(m_MotorLeft);
    m_MotorRightFollow.follow(m_MotorRight);
    }*/


// Speed changer methods :

    /*public void reverse(){
      direction = -direction;
    }*/

    public void speedUp(){
      if(speed_changer <= 0.8){
      speed_changer = speed_changer + 0.3;
      }
    }
    public void speedDown(){
      if(speed_changer >= 0.1){
      speed_changer = speed_changer - 0.3;
      }
    }

    public void setMaxOutput(double speed_changer) {
      m_drive.setMaxOutput(speed_changer);
      }

// Sensors methods :

    public double getAngle(){
      return gyro.getAngle();
    }
    public double getRate(){
      return gyro.getRate();
    }
    public void resetGyro(){
      gyro.reset();
    } 
    /*public void calibrateGyro(){
      gyro.calibrate();
    }*/

    public void zeroHeading() {
      gyro.reset();
    }
  
    public double getHeading() {
      return gyro.getRotation2d().getDegrees();
    }
  
  
    public double getTurnRate() {
      return -gyro.getRate();
    }

    public double getRightPosition(){
      return encoderDriveR.get();
    }
    
    public double getLeftPosition(){
      return encoderDriveL.get();
    }
    
    public double getRightDistance(){
      return encoderDriveR.getDistance();
    }
    
    public double getLeftDistance(){
      return encoderDriveL.getDistance();
    }
    
    public void resetEncoders() {
      encoderDriveL.reset();
      encoderDriveR.reset();
    }

    public double getAverageEncoderDistance() {
      return (encoderDriveL.getDistance() + encoderDriveR.getDistance()) / 2.0;
    }
  
// Odometry methods :
    
   public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(encoderDriveL.getRate(), encoderDriveR.getRate());
  }

  /*public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        gyro.getRotation2d(), encoderDriveL.getDistance(), encoderDriveR.getDistance(), pose);
  }*/

  
@Override
  public void periodic() {

    /*m_odometry.update(
      gyro.getRotation2d(), encoderDriveL.getDistance(), encoderDriveR.getDistance());

    SmartDashboard.putNumber("Left encoder value meters", encoderDriveL.getDistance());
    SmartDashboard.putNumber("Right encoder value meters", encoderDriveR.getDistance());
    SmartDashboard.putNumber("Gyro angle", gyro.getAngle());
    SmartDashboard.putNumber("Rate L", encoderDriveL.getRate());
    SmartDashboard.putNumber("Rate R", encoderDriveR.getRate());*/

  } // This method will be called once per scheduler run


  

}
