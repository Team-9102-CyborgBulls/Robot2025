package frc.robot.commands.ElevatorCmd;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ElevatorUpCmd extends Command {
    
    ElevatorSubsystem m_elevator;

    public ElevatorUpCmd(ElevatorSubsystem elevator){

      this.m_elevator = elevator;
      addRequirements(m_elevator);
    }

    @Override
  public void initialize() {
    
    RobotContainer.m_timer.reset();
    RobotContainer.m_timer.start();

  }

  @Override
  public void execute() {
    m_elevator.setElevatorMotor(0.8);
  }

  @Override
  public boolean isFinished() {
    
    return false;
  }
  

  @Override
  public void end(boolean interrupted){
    m_elevator.stop();
  }
}
