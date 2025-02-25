package frc.robot.commands.IntakeCmd;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;


public class IntakeUpCmd extends Command {
    
    IntakeSubsystem m_intake;

    public IntakeUpCmd(IntakeSubsystem intake){

      this.m_intake = intake;
      addRequirements(m_intake);
    }

    @Override
  public void initialize() {
    
    RobotContainer.m_timer.reset();
    RobotContainer.m_timer.start();

  }

  @Override
  public void execute() {
    m_intake.setIntakeAngle(1);
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
    m_intake.setIntakeZero();
  }
}
