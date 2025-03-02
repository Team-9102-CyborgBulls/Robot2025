// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.OutTakeCmd;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.OutTakeAlgueSubsystem;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class OutTakeAlgueCmd extends Command {

  OutTakeAlgueSubsystem m_outTakeAlgue;
  /** Creates a new OutTakeAlgueCmd. */
  public OutTakeAlgueCmd(OutTakeAlgueSubsystem outTakeAlgueSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_outTakeAlgue = outTakeAlgueSubsystem;
    addRequirements(m_outTakeAlgue);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_outTakeAlgue.setOutTakeAlgueMotor(0.8);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_outTakeAlgue.setOutTakeAlgueMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*if(m_outTakeAlgue.limitSwitchActivated()){
      return true;
    }
    else{
      return false;
    }*/
    return false;
  }
}
