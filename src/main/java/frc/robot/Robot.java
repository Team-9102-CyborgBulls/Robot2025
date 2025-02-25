// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    
    m_robotContainer = new RobotContainer();
    m_robotContainer.driveSubsystem.zeroHeading();
    m_robotContainer.driveSubsystem.resetEncoders();

    SmartDashboard.putData(m_robotContainer.driveSubsystem.gyro);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.

    SmartDashboard.putNumber("Encoder Right",m_robotContainer.driveSubsystem.getRightDistance());
    SmartDashboard.putNumber("Encoder Left",m_robotContainer.driveSubsystem.getLeftDistance());
    SmartDashboard.putNumber("Encoder R Rate",m_robotContainer.driveSubsystem.getRate());
    //SmartDashboard.putNumber("TargetSpeed",m_robotContainer.driveSubsystem.getTargetSpeed(m_robotContainer.manette.getLeftY(),0));
    CommandScheduler.getInstance().run();

    //m_robotContainer.driveSubsystem.setFollowers();
  }

  
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    m_robotContainer.driveSubsystem.zeroHeading();
    m_robotContainer.driveSubsystem.resetEncoders();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.driveSubsystem.zeroHeading();
    m_robotContainer.driveSubsystem.resetEncoders();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
   // m_robotContainer.driveSubsystem.setFollowers();
  }

 
}
