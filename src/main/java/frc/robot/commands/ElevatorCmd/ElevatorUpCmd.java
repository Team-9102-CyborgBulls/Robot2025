package frc.robot.commands.ElevatorCmd;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ElevatorUpCmd extends Command {
    
    ElevatorSubsystem m_elevator;
    double setpoint;

    public ElevatorUpCmd(ElevatorSubsystem elevator,double setpoint){

      this.m_elevator = elevator;
      this.setpoint = setpoint;
      addRequirements(m_elevator);
    }

    @Override
  public void initialize() {
    
    RobotContainer.m_timer.reset();
    RobotContainer.m_timer.start();

  }

  @Override
  public void execute() {
    double error = setpoint - m_elevator.getEncoderValue();
    double speed = (Constants.ElevatorConstants.kp * error) + Constants.ElevatorConstants.kg;

    if(speed >= 0.8){
      m_elevator.setElevatorMotor(0.8);
    
    } else{
      m_elevator.setElevatorMotor(speed);
    }
    
  }

  @Override
  public boolean isFinished() {
    if(m_elevator.getEncoderValue() >= RobotContainer.setelevator + 3){
      return true;
    }else{
      return false;
    }
    
  }
  

  @Override
  public void end(boolean interrupted){
    m_elevator.setElevatorMotor(Constants.ElevatorConstants.kg);;
  }
}
