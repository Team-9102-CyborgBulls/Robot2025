// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.commands.AutoCmd.Auto1CoralM;
import frc.robot.commands.GettingInRangeCmd;
import frc.robot.commands.IntakeCmd;
import frc.robot.commands.NothingCmd;
import frc.robot.commands.OutTakeCmd;
import frc.robot.commands.AutoCmd.Auto1CoralL;
import frc.robot.commands.AutoCmd.Auto1CoralR;
import frc.robot.commands.DriveCmd.DriveCmd;
import frc.robot.commands.DriveCmd.DriveForDistanceCmd;
import frc.robot.commands.DriveCmd.TurnToAngleCmd;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OutTakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;


public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final static DriveSubsystem driveSubsystem = new DriveSubsystem();
  public final DriveCmd driveCmd = new DriveCmd(driveSubsystem);

  public final static VisionSubsystem visionsubsystem = new VisionSubsystem();
  public final GettingInRangeCmd gettinginrange = new GettingInRangeCmd(driveSubsystem, visionsubsystem);

  public final OutTakeSubsystem outTakeSubsystem = new OutTakeSubsystem();
  public final OutTakeCmd outTakeCmd = new OutTakeCmd(outTakeSubsystem);

  public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public final IntakeCmd intakeCmd = new IntakeCmd(intakeSubsystem);
  
  public static CommandXboxController manette = new CommandXboxController(0);
  public static Timer m_timer = new Timer();

  public SendableChooser<Command> m_Chooser = new SendableChooser<Command>();

  public final DriveForDistanceCmd Drive1m = new DriveForDistanceCmd(1);
  public final TurnToAngleCmd Turn90 = new TurnToAngleCmd(driveSubsystem,90);
  public final Auto1CoralR auto1CoralR = new Auto1CoralR();
  public final Auto1CoralM auto1CoralM = new Auto1CoralM();
  public final Auto1CoralL auto1CoralL = new Auto1CoralL();

  public final NothingCmd nothingCmd = new NothingCmd(driveSubsystem);
 
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // Add commands to the autonomous command chooser
    m_Chooser.setDefaultOption("Auto Test",Drive1m );

    m_Chooser.addOption("Auto Turn", Turn90);
    m_Chooser.addOption("Auto R", auto1CoralR);
    m_Chooser.addOption("Auto M", auto1CoralM);
    m_Chooser.addOption("Auto L", auto1CoralL);
    

    //Put the chooser on the dashboard
    SmartDashboard.putData(m_Chooser);

    
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

   
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    Trigger rBumper = manette.rightBumper();
    Trigger lBumper = manette.leftBumper();

    Trigger UpButton = manette.povUp();
    Trigger DownButton = manette.povDown();
    Trigger LeftButton = manette.povLeft();
    Trigger RightButton = manette.povRight();

    Trigger startButton = manette.start();
    
    DoubleSupplier leftY = () -> manette.getLeftY();
      DoubleSupplier rightX = () -> manette.getRightX();
     
    driveSubsystem.setDefaultCommand(new InstantCommand(() -> 
      driveSubsystem.teleopDriveCommand(leftY, rightX),driveSubsystem
  ));
    /*driveSubsystem.setDefaultCommand(
        driveSubsystem.arcadeDriveCommand(
            () -> manette.getLeftY(), () -> manette.getRightX()));*/

    /*manette
        .a()
        
        .whileTrue(driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    manette
        .b()
        
        .whileTrue(driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    manette
        .x()
        
        .whileTrue(driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    manette
        .y()
        
        .whileTrue(driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));*/

    manette.a().whileTrue(new OutTakeCmd(outTakeSubsystem));
    manette.b().onTrue(new IntakeCmd(intakeSubsystem));

    UpButton.whileTrue(new InstantCommand(()-> driveSubsystem.driveRight(0.5))).whileFalse(new InstantCommand(()-> driveSubsystem.driveRight(0)));
    DownButton.whileTrue(new InstantCommand(()-> driveSubsystem.driveRightFollow(0.3))).whileFalse(new InstantCommand(()-> driveSubsystem.driveRightFollow(0)));
    LeftButton.whileTrue(new InstantCommand(()-> driveSubsystem.driveLeft(0.5))).whileFalse(new InstantCommand(()-> driveSubsystem.driveLeft(0)));
    RightButton.whileTrue(new InstantCommand(()-> driveSubsystem.driveLeftFollow(0.3))).whileFalse(new InstantCommand(()-> driveSubsystem.driveLeftFollow(0)));

    startButton.whileTrue(new GettingInRangeCmd(driveSubsystem,visionsubsystem));

    rBumper.onTrue(new InstantCommand(() -> driveSubsystem.speedUp())); // Vitesse augmenté
    lBumper.onTrue(new InstantCommand(() -> driveSubsystem.speedDown())); // vitesse baissé

    //intakeSubsystem.setDefaultCommand(intakeSubsystem.ServoDefaultCmd(intakeSubsystem, 1));
    // Schedule exampleMethodCommand when the Xbox controller's B button is pressed,
    // cancelling on release.
    
  
  }

  /**
   * 
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create a voltage constraint to ensure we don't accelerate too fast
    return m_Chooser.getSelected();
}
}
