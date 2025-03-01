package frc.robot.commands.ElevatorCmd;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeAlgueSubsystem;

public class ElevatorDownCmd extends Command{
    

    ElevatorSubsystem m_elevator;
    double setpoint;

    public ElevatorDownCmd(ElevatorSubsystem elevator,double setpoint){

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

    if(speed <= -0.7){
      m_elevator.setElevatorMotor(-0.7);
    }else{
    m_elevator.setElevatorMotor(speed); // value déjà négative car setpoit < encoderValue
  }
}

  @Override
  public boolean isFinished() {
    if(m_elevator.getEncoderValue() <= Constants.ElevatorConstants.ELEVATOR_DOWN_POSITION){
      return true;
    }else{
      return false;
    }
    
  }
  

  @Override
  public void end(boolean interrupted){
    m_elevator.setElevatorMotor(Constants.ElevatorConstants.kg);
  }
}
