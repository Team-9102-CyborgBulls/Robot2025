// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.commands.AutoCmd.Auto1CoralM;
import frc.robot.commands.GettingInRangeCmd;
import frc.robot.commands.NothingCmd;
import frc.robot.commands.OutTakeAlgueCmd;
import frc.robot.commands.OutTakeCmd;
import frc.robot.commands.ArmCmd.ArmDownCmd;
import frc.robot.commands.ArmCmd.ArmUpCmd;
import frc.robot.commands.ArmCmd.IntakeArmCmd;
import frc.robot.commands.ArmCmd.IntakeArmReverseCmd;
import frc.robot.commands.ArmCmd.ArmStillCmd;
import frc.robot.commands.AutoCmd.Auto1CoralL;
import frc.robot.commands.AutoCmd.Auto1CoralR;
import frc.robot.commands.DriveCmd.DriveCmd;
import frc.robot.commands.DriveCmd.DriveForDistanceCmd;
import frc.robot.commands.DriveCmd.TurnToAngleCmd;
import frc.robot.commands.ElevatorCmd.ElevatorDownCmd;
import frc.robot.commands.ElevatorCmd.ElevatorStillCmd;
import frc.robot.commands.ElevatorCmd.ElevatorUpCmd;
import frc.robot.commands.IntakeCmd.IntakeDownCmd;
import frc.robot.commands.IntakeCmd.IntakeUpCmd;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OutTakeAlgueSubsystem;
import frc.robot.subsystems.OutTakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;


public class RobotContainer {
  boolean manuel = false;
  // The robot's subsystems and commands are defined here...
  public final static DriveSubsystem driveSubsystem = new DriveSubsystem();
  public final DriveCmd driveCmd = new DriveCmd(driveSubsystem);

  public final static VisionSubsystem visionsubsystem = new VisionSubsystem();
  public final GettingInRangeCmd gettinginrange = new GettingInRangeCmd(driveSubsystem, visionsubsystem);

  public final OutTakeSubsystem outTakeSubsystem = new OutTakeSubsystem();
  public final OutTakeCmd outTakeCmd = new OutTakeCmd(outTakeSubsystem);

  public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public final IntakeUpCmd intakeUpCmd = new IntakeUpCmd(intakeSubsystem);
  public final IntakeDownCmd intakeDownCmd = new IntakeDownCmd(intakeSubsystem);

  public static double setelevator = Constants.ElevatorConstants.ELEVATOR_L2_POSITION;
  //public double setpoint = 24;
  public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  public final ElevatorDownCmd elevatorDownCmd = new ElevatorDownCmd(elevatorSubsystem,Constants.ElevatorConstants.ELEVATOR_DOWN_POSITION);
  public final ElevatorStillCmd elevatorStillCmd = new ElevatorStillCmd(elevatorSubsystem);
  public final ElevatorUpCmd elevatorUpCmd = new ElevatorUpCmd(elevatorSubsystem,setelevator);
  

  public final ArmSubsystem armSubsystem = new ArmSubsystem();
  public final ArmDownCmd armDownCmd = new ArmDownCmd(armSubsystem);
  public final ArmUpCmd armUpCmd = new ArmUpCmd(armSubsystem);
  public final ArmStillCmd armStillCmd = new ArmStillCmd(armSubsystem);
  public final IntakeArmCmd intakeArmCmd = new IntakeArmCmd(armSubsystem);
  public final IntakeArmReverseCmd intakeArmReverseCmd = new IntakeArmReverseCmd(armSubsystem);

  public final OutTakeAlgueSubsystem OutTakeAlgueSubsystem = new OutTakeAlgueSubsystem();
  public final OutTakeAlgueCmd outTakeAlgueCmd = new OutTakeAlgueCmd(OutTakeAlgueSubsystem);
  
  public UsbCamera drivercamera = CameraServer.startAutomaticCapture();

   public PhotonCamera camera = new PhotonCamera("photonvision");


  
  public static CommandXboxController manette = new CommandXboxController(0);
  public static Joystick k_joystick = new Joystick(1);
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

    JoystickButton button1 = new JoystickButton(k_joystick, 1);
    JoystickButton button2 = new JoystickButton(k_joystick, 2);
    
     
    /*driveSubsystem.setDefaultCommand(new InstantCommand(() -> 
      driveSubsystem.teleopDriveCommand(() -> manette.getLeftY(),() -> manette.getRightX()), driveSubsystem)
  );*/
    driveSubsystem.setDefaultCommand(driveCmd);

    elevatorSubsystem.setDefaultCommand(elevatorStillCmd);

    armSubsystem.setDefaultCommand(armStillCmd);

    //intakeSubsystem.setDefaultCommand(intakeUpCmd);
        

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
    if(manuel==false){
      manette.a().whileTrue(new OutTakeCmd(outTakeSubsystem));
      manette.b().onTrue(new IntakeDownCmd(intakeSubsystem));

      manette.y().whileTrue(new IntakeArmCmd(armSubsystem));
      manette.x().whileTrue(new IntakeArmReverseCmd(armSubsystem));

      //UpButton.whileTrue(new InstantCommand(()-> driveSubsystem.driveRight(0.5))).whileFalse(new InstantCommand(()-> driveSubsystem.driveRight(0)));
      //DownButton.whileTrue(new InstantCommand(()-> driveSubsystem.driveRightFollow(0.3))).whileFalse(new InstantCommand(()-> driveSubsystem.driveRightFollow(0)));

      UpButton.onTrue(new ElevatorUpCmd(elevatorSubsystem,setelevator));
      DownButton.onTrue(new ElevatorDownCmd(elevatorSubsystem,Constants.ElevatorConstants.ELEVATOR_DOWN_POSITION));

      LeftButton.whileTrue(new ArmUpCmd(armSubsystem));
      RightButton.whileTrue(new ArmDownCmd(armSubsystem));
      //LeftButton.whileTrue(new InstantCommand(()-> driveSubsystem.driveLeft(0.5))).whileFalse(new InstantCommand(()-> driveSubsystem.driveLeft(0)));
      //RightButton.whileTrue(new InstantCommand(()-> driveSubsystem.driveLeftFollow(0.3))).whileFalse(new InstantCommand(()-> driveSubsystem.driveLeftFollow(0)));

      rBumper.onTrue(new InstantCommand(() -> driveSubsystem.speedUp())); // Vitesse augmenté
      lBumper.onTrue(new InstantCommand(() -> driveSubsystem.speedDown())); // vitesse baissé
    }
    else if (manuel == true){
      manette.a().whileTrue(new OutTakeCmd(outTakeSubsystem));
      manette.b().onTrue(new IntakeDownCmd(intakeSubsystem));

      manette.y().whileTrue(new IntakeArmCmd(armSubsystem));
      manette.x().whileTrue(new IntakeArmReverseCmd(armSubsystem));

      //UpButton.whileTrue(new InstantCommand(()-> driveSubsystem.driveRight(0.5))).whileFalse(new InstantCommand(()-> driveSubsystem.driveRight(0)));
      //DownButton.whileTrue(new InstantCommand(()-> driveSubsystem.driveRightFollow(0.3))).whileFalse(new InstantCommand(()-> driveSubsystem.driveRightFollow(0)));

      UpButton.whileTrue(new ElevatorUpCmd(elevatorSubsystem,setelevator));
      DownButton.whileTrue(new ElevatorDownCmd(elevatorSubsystem,Constants.ElevatorConstants.ELEVATOR_DOWN_POSITION));

      LeftButton.whileTrue(new ArmUpCmd(armSubsystem));
      RightButton.whileTrue(new ArmDownCmd(armSubsystem));
      //LeftButton.whileTrue(new InstantCommand(()-> driveSubsystem.driveLeft(0.5))).whileFalse(new InstantCommand(()-> driveSubsystem.driveLeft(0)));
      //RightButton.whileTrue(new InstantCommand(()-> driveSubsystem.driveLeftFollow(0.3))).whileFalse(new InstantCommand(()-> driveSubsystem.driveLeftFollow(0)));

      rBumper.onTrue(new InstantCommand(() -> driveSubsystem.speedUp())); // Vitesse augmenté
      lBumper.onTrue(new InstantCommand(() -> driveSubsystem.speedDown())); // vitesse baissé
    }

    if (k_joystick.getRawButtonPressed(7) == true){
      System.out.println("debug");
      setelevator = elevatorSubsystem.changesetpoint(setelevator);
    }

    button1.onTrue(new InstantCommand(() -> {
      System.out.println("debug");
      setelevator = elevatorSubsystem.changesetpoint(setelevator);}));

      button2.onTrue(new InstantCommand(() -> {
        System.out.println("deb");
        manuel = !manuel; // Inverse l'état de manuel (toggle)
    }));
    
  
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

