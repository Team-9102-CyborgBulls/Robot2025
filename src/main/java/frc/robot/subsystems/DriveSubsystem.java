// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.SerialPort;

 
public class DriveSubsystem extends SubsystemBase {
  
     
     private DifferentialDriveOdometry m_odometry;
     public double direction = 1.0;
     public double speed_changer = 0.6;

    WPI_TalonSRX m_MotorRight = new WPI_TalonSRX(1);
    WPI_TalonSRX m_MotorRightFollow = new WPI_TalonSRX(3);
    WPI_TalonSRX m_MotorLeftFollow = new WPI_TalonSRX(4);
    WPI_TalonSRX m_MotorLeft = new WPI_TalonSRX(2);

    public AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

    
    private final DifferentialDrive m_drive =
    new DifferentialDrive(m_MotorRight::set, m_MotorLeft::set);

    public static Encoder encoderDriveR = new Encoder(0,1, false);
    public static Encoder encoderDriveL = new Encoder(2,3,true);

    private static double lastPositionR;
    private static double lastPositionL;
    private static double lastTime;
            
    static  int kEncoderCPR = 8192;
    static double kWheelDiameterMeters = Units.inchesToMeters(6);
    static double kEncoderDistancePerPulse = (kWheelDiameterMeters * Math.PI) / kEncoderCPR;

    

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
            new SysIdRoutine.Config(),
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
                                              
                                                
  m_MotorRight.setInverted(true);
  m_MotorRightFollow.setInverted(true);
  m_MotorLeft.setInverted(false);
  m_MotorLeftFollow.setInverted(false);
                                            
  SendableRegistry.addChild(m_drive, m_MotorLeft);
  SendableRegistry.addChild(m_drive, m_MotorRight);

  m_MotorLeftFollow.follow(m_MotorLeft);
  m_MotorRightFollow.follow(m_MotorRight);
                                           
  /*m_MotorRight.configVoltageCompSaturation(11.0);
  m_MotorRightFollow.configVoltageCompSaturation(11.0);
  m_MotorLeft.configVoltageCompSaturation(11.0);
  m_MotorLeftFollow.configVoltageCompSaturation(11.0);

  m_MotorRight.configContinuousCurrentLimit(40); // Limite continue de 40A
  m_MotorRight.enableCurrentLimit(true); // Active la limitation

  m_MotorRightFollow.configContinuousCurrentLimit(40); // Limite continue de 40A
  m_MotorRightFollow.enableCurrentLimit(true); // Active la limitation

  m_MotorLeft.configContinuousCurrentLimit(40); // Limite continue de 40A
  m_MotorLeft.enableCurrentLimit(true); // Active la limitation

  m_MotorLeftFollow.configContinuousCurrentLimit(40); // Limite continue de 40A
  m_MotorLeftFollow.enableCurrentLimit(true); // Active la limitation
                                              
  m_MotorRight.setSafetyEnabled(true);
  m_MotorRightFollow.setSafetyEnabled(true);
  m_MotorLeft.setSafetyEnabled(true);
  m_MotorLeftFollow.setSafetyEnabled(true);*/
  


  encoderDriveL.setDistancePerPulse(kEncoderDistancePerPulse);
  encoderDriveR.setDistancePerPulse(kEncoderDistancePerPulse);
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
    AutoBuilder.configure(
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
    );
                                                                                          
}
                                              
  
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }


  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
      return m_sysIdRoutine.dynamic(direction);
  }

    /*public void arcadeDrive(double fwd, double rot) {
      m_drive.arcadeDrive(fwd*direction, rot);
      m_drive.setMaxOutput(speed_changer);
       m_MotorRightFollow.follow(m_MotorRight);
      m_MotorLeftFollow.follow(m_MotorLeft);
    }*/
    
    public Command arcadeDriveCommand(DoubleSupplier fwd, DoubleSupplier rot) {
    // A split-stick arcade command, with forward/backward controlled by the left
    // hand, and turning controlled by the right.
    return run(() -> m_drive.arcadeDrive(-fwd.getAsDouble(), rot.getAsDouble()))
        .withName("arcadeDrive");
  }

    public void tankDrive(double left, double right){
      m_drive.tankDrive(left, right);
    }

    public void setMaxOutput(double speed_changer) {
      m_drive.setMaxOutput(speed_changer);
    }

    public void reverse(){
      direction = -direction;
    }

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
      gyro.
    }*/
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
    
    public void resetEncoder() {
      encoderDriveL.reset();
      encoderDriveR.reset();
    }
   
   
    
  @Override
  public void periodic() {

    m_odometry.update(
      gyro.getRotation2d(), encoderDriveL.getDistance(), encoderDriveR.getDistance());

  } // This method will be called once per scheduler run

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(encoderDriveL.getRate(), encoderDriveR.getRate());
  }

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        gyro.getRotation2d(), encoderDriveL.getDistance(), encoderDriveR.getDistance(), pose);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_MotorLeft.setVoltage(leftVolts);
    m_MotorRight.setVoltage(rightVolts);
    m_drive.feed();
  }

  public void resetEncoders() {
    encoderDriveL.reset();
    encoderDriveR.reset();
  }

  public double getAverageEncoderDistance() {
    return (encoderDriveL.getDistance() + encoderDriveR.getDistance()) / 2.0;
  }

  public void zeroHeading() {
    gyro.reset();
  }

  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
  }

  public double getTurnRate() {
    return -gyro.getRate();
  }

  public void setDriveMotors(double forward, double turn){

    double left = forward - turn;
    double right = forward + turn;

    m_MotorRight.set(TalonSRXControlMode.PercentOutput, right);
    m_MotorLeft.set(TalonSRXControlMode.PercentOutput, left);

    m_MotorRightFollow.set(TalonSRXControlMode.PercentOutput, right);
    m_MotorLeftFollow.set(TalonSRXControlMode.PercentOutput, left);
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


  

}
