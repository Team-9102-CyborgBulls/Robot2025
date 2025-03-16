// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.commands.AutoCmd.Auto1CoralMidle;
import frc.robot.commands.AutoCmd.AutoTurn;
import frc.robot.commands.GettingInRangeCmd;
import frc.robot.commands.IntakeAlgueCmd;
import frc.robot.commands.IntakeAlgueReverseCmd;
import frc.robot.commands.IntakeCoralCmd;
import frc.robot.commands.NothingCmd;
import frc.robot.commands.ArmCmd.ArmDownCmd;
import frc.robot.commands.ArmCmd.ArmUpCmd;

import frc.robot.commands.ArmCmd.ArmStillCmd;
import frc.robot.commands.AutoCmd.Auto1CoralAllianceBleuCageBleu;
import frc.robot.commands.AutoCmd.Auto1CoralAllianceBleuCageRouge;
import frc.robot.commands.AutoCmd.Auto1CoralAllianceRougeCageBleu;
import frc.robot.commands.AutoCmd.Auto1CoralAllianceRougeCagesRouges;
import frc.robot.commands.DriveCmd.DriveCmd;
import frc.robot.commands.DriveCmd.DriveForDistanceCmd;
import frc.robot.commands.DriveCmd.TurnToAngleCmd;
import frc.robot.commands.ElevatorCmd.ElevatorDownCmd;
import frc.robot.commands.ElevatorCmd.ElevatorDownManualCmd;
import frc.robot.commands.ElevatorCmd.ElevatorStillCmd;
import frc.robot.commands.ElevatorCmd.ElevatorUpCmd;
import frc.robot.commands.ElevatorCmd.ElevatorUpManualCmd;
import frc.robot.commands.OutTakeCmd.OutTakeCmd;
import frc.robot.commands.OutTakeCmd.OutTakeAlgueCmd;
import frc.robot.commands.OutTakeCmd.OutTakeAlgueManualCmd;
import frc.robot.commands.OutTakeCmd.OutTakeAutoCmd;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeAlgueSubsystem;
import frc.robot.subsystems.OutTakeAlgueSubsystem;
import frc.robot.subsystems.OutTakeCoralSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
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




public class RobotContainer {

  public boolean useManette1 = false;
  public boolean lastButtonState = false; // Pour détecter l’appui du bouton
  // The robot's subsystems and commands are defined here...
  public final static DriveSubsystem driveSubsystem = new DriveSubsystem();
  public final DriveCmd driveCmd = new DriveCmd(driveSubsystem);

  public final static VisionSubsystem visionsubsystem = new VisionSubsystem();
  public final GettingInRangeCmd gettinginrange = new GettingInRangeCmd(driveSubsystem, visionsubsystem);

  public final static OutTakeCoralSubsystem outTakeSubsystem = new OutTakeCoralSubsystem();
  public final OutTakeCmd outTakeCmd = new OutTakeCmd(outTakeSubsystem);


  public final IntakeAlgueSubsystem intakeAlgueSubsystem = new IntakeAlgueSubsystem();
  public final IntakeAlgueCmd intakeUpCmd = new IntakeAlgueCmd(intakeAlgueSubsystem);
  public final IntakeAlgueCmd intakeDownCmd = new IntakeAlgueCmd(intakeAlgueSubsystem);

  //public  double setelevator = Constants.ElevatorConstants.ELEVATOR_L2_POSITION;
  //public double setpoint = 24;
  public final static ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  public final ElevatorDownCmd elevatorDownCmd = new ElevatorDownCmd(elevatorSubsystem);
  public final ElevatorStillCmd elevatorStillCmd = new ElevatorStillCmd(elevatorSubsystem);
  public final ElevatorUpCmd elevatorUpCmd = new ElevatorUpCmd(elevatorSubsystem);
  public final ElevatorDownManualCmd elevatorDownManualCmd = new ElevatorDownManualCmd(elevatorSubsystem);
  public final ElevatorUpManualCmd elevatorUpManualCmd = new ElevatorUpManualCmd(elevatorSubsystem);

  public final ArmSubsystem armSubsystem = new ArmSubsystem();
  public final ArmDownCmd armDownCmd = new ArmDownCmd(armSubsystem);
  public final ArmUpCmd armUpCmd = new ArmUpCmd(armSubsystem);
  public final ArmStillCmd armStillCmd = new ArmStillCmd(armSubsystem);
  
  
  public final OutTakeAlgueSubsystem outTakeAlgueSubsystem = new OutTakeAlgueSubsystem();
  public final OutTakeAlgueCmd outTakeAlgueCmd = new OutTakeAlgueCmd(outTakeAlgueSubsystem);
  public final IntakeCoralCmd intakeCoralCmd = new IntakeCoralCmd(outTakeSubsystem);
  
  
  public UsbCamera drivercamera = CameraServer.startAutomaticCapture();
  public PhotonCamera camera = new PhotonCamera("photonvision");

  public static CommandXboxController manette = new CommandXboxController(0);
  public static Joystick k_joystick = new Joystick(1);
  public static CommandXboxController manette2 = new CommandXboxController(2);
  public static Timer m_timer = new Timer();

  public SendableChooser<Command> m_Chooser = new SendableChooser<Command>();

  public final DriveForDistanceCmd Drive0_3m = new DriveForDistanceCmd(0.3);
  public final TurnToAngleCmd Turn90 = new TurnToAngleCmd(driveSubsystem,90);

  public final Auto1CoralAllianceRougeCagesRouges auto1CoralRR = new Auto1CoralAllianceRougeCagesRouges();
  public final Auto1CoralMidle auto1CoralM = new Auto1CoralMidle();
  public final Auto1CoralAllianceBleuCageRouge auto1CoralBR = new Auto1CoralAllianceBleuCageRouge();
  public final Auto1CoralAllianceRougeCageBleu auto1CoralRB = new Auto1CoralAllianceRougeCageBleu();
  public final Auto1CoralAllianceBleuCageBleu auto1CoralBB = new Auto1CoralAllianceBleuCageBleu();

  public final NothingCmd nothingCmd = new NothingCmd(driveSubsystem);

  public final AutoTurn turn = new AutoTurn();
 
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // Add commands to the autonomous command chooser
    m_Chooser.setDefaultOption("Auto Test",Drive0_3m );

    //m_Chooser.addOption("Auto Turn", Turn90);
    m_Chooser.addOption("Auto A Rouge C Rouge", auto1CoralRR);
    m_Chooser.addOption("Auto Midle", auto1CoralM);
    m_Chooser.addOption("Auto A Rouge C Bleu", auto1CoralRB);
    m_Chooser.addOption("Auto A Bleu C Bleu", auto1CoralBB);
    m_Chooser.addOption("Auto A Bleu C Rouge", auto1CoralBR);
    m_Chooser.addOption("Ligne", Drive0_3m);
    m_Chooser.addOption("turn", turn);
    

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

    Trigger UpButton2 = manette2.povUp();
    Trigger DownButton2 = manette2.povDown();
    Trigger LeftButton2 = manette2.povLeft();
    Trigger RightButton2 = manette2.povRight();

    Trigger rBumper2 = manette2.rightBumper();
    Trigger lBumper2 = manette2.leftBumper();



    
    
    driveSubsystem.setDefaultCommand(driveCmd);

    elevatorSubsystem.setDefaultCommand(elevatorStillCmd);

    //armSubsystem.setDefaultCommand(armStillCmd);

    //intakeSubsystem.setDefaultCommand(intakeUpCmd);
        
    

     /*  new JoystickButton(k_joystick, 8)
        .onTrue(new InstantCommand(() -> {
            
            manuel = !manuel; // Inverse l'état de manuel (toggle)
        }));
      */
        new JoystickButton(k_joystick, 7)
        .whileTrue(new InstantCommand(() -> {
            
            elevatorSubsystem.setelevator = elevatorSubsystem.changesetpoint(elevatorSubsystem.setelevator);
        }));
       

        new JoystickButton(k_joystick, 2)
        .whileTrue(new ArmUpCmd(armSubsystem));
       
        new JoystickButton(k_joystick, 1)
        .whileTrue(new ArmDownCmd(armSubsystem));

        new JoystickButton(k_joystick, 11)
        .whileTrue(new OutTakeAlgueManualCmd(outTakeAlgueSubsystem)); 
            
        new JoystickButton(k_joystick, 12).whileTrue(new IntakeAlgueReverseCmd(intakeAlgueSubsystem));
           
       


  manette.a().whileTrue(new OutTakeCmd(outTakeSubsystem));
  manette.b().whileTrue(new IntakeCoralCmd(outTakeSubsystem));

  manette.y().onTrue(new IntakeAlgueCmd(intakeAlgueSubsystem));
  manette.x().whileTrue(new OutTakeAlgueCmd(outTakeAlgueSubsystem));

  manette.back().whileTrue(new InstantCommand(()->driveSubsystem.reverse()));

  UpButton.onTrue(new ElevatorUpCmd(elevatorSubsystem));
  DownButton.onTrue(new ElevatorDownCmd(elevatorSubsystem));

  LeftButton.onTrue(new ArmUpCmd(armSubsystem));
  RightButton.onTrue(new ArmDownCmd(armSubsystem));

  rBumper.onTrue(new InstantCommand(() -> driveSubsystem.speedUp())); // Vitesse augmenté
  lBumper.onTrue(new InstantCommand(() -> driveSubsystem.speedDown())); // vitesse baissé
    
    

    
      
     
  manette2.a().whileTrue(new OutTakeCmd(outTakeSubsystem));
  manette2.b().whileTrue(new IntakeCoralCmd(outTakeSubsystem));

  manette2.y().whileTrue(new IntakeAlgueCmd(intakeAlgueSubsystem));
  manette2.x().whileTrue(new OutTakeAlgueCmd(outTakeAlgueSubsystem));

    
    
  manette2.back().whileTrue(new InstantCommand(()->driveSubsystem.reverse()));
  
  UpButton2.whileTrue(new ElevatorUpManualCmd(elevatorSubsystem));
  DownButton2.whileTrue(new ElevatorDownManualCmd(elevatorSubsystem));

  LeftButton2.whileTrue(new ArmUpCmd(armSubsystem));
  RightButton2.whileTrue(new ArmDownCmd(armSubsystem));
      
  rBumper2.onTrue(new InstantCommand(() -> driveSubsystem.speedUp())); // Vitesse augmenté
  lBumper2.onTrue(new InstantCommand(() -> driveSubsystem.speedDown())); // vitesse baissé
    
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

