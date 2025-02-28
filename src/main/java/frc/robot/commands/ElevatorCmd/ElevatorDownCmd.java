package frc.robot.commands.ElevatorCmd;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ElevatorDownCmd extends Command{
    IntakeSubsystem m_intake;

    ElevatorSubsystem m_elevator;

    public ElevatorDownCmd(ElevatorSubsystem elevator){

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
    m_elevator.setElevatorMotor(-0.7);
  }

  @Override
  public boolean isFinished() {
    if(m_elevator.cypher.get() <= Constants.ElevatorConstants.ELEVATOR_DOWN_POSITION){
      return true;
    }
    //else if(m_elevator.cypher.get() <= Constants.ElevatorConstants.ELEVATOR_DOWN_POSITION)
    return false;
  }
  

  @Override
  public void end(boolean interrupted){
    m_elevator.stop();
  }
}
