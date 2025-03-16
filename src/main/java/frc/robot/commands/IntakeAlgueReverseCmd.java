package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeAlgueSubsystem;
import frc.robot.subsystems.OutTakeAlgueSubsystem;


public class IntakeAlgueReverseCmd extends Command {
    
    IntakeAlgueSubsystem m_intake;

    public IntakeAlgueReverseCmd(IntakeAlgueSubsystem intakeAlgueSubsystem){

      this.m_intake = intakeAlgueSubsystem;
      addRequirements(m_intake);
    }

    @Override
  public void initialize() {
    
    RobotContainer.m_timer.reset();
    RobotContainer.m_timer.start();

  }

  @Override
  public void execute() {
    m_intake.setIntakeAlgueMotor(-0.3);
  }

  @Override
  public boolean isFinished() {
    if(RobotContainer.m_timer.get() >= 2){ //a définir
      return true;
  }
  else{
    return false;
  }
  }

  @Override
  public void end(boolean interrupted){
    m_intake.setIntakeAlgueMotor(0);
  }
}
