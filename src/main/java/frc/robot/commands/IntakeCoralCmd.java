package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.OutTakeCoralSubsystem;

public class IntakeCoralCmd extends Command {
    
    OutTakeCoralSubsystem m_outTake;

    public IntakeCoralCmd(OutTakeCoralSubsystem outTake){

      this.m_outTake = outTake;
      addRequirements(m_outTake);
    }

    @Override
  public void initialize() {
    
    RobotContainer.m_timer.reset();
    RobotContainer.m_timer.start();

  }

  @Override
  public void execute() {
    m_outTake.setOutTakeMotor(-0.3);
  }

  @Override
  public boolean isFinished() {
     if(m_outTake.intake_sensor.get()){ //a définir
      return true;
  } 
  else{
    return false;
  }
  }

  @Override
  public void end(boolean interrupted){
    m_outTake.stop();
  }
}
