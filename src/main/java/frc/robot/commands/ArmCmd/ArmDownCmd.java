package frc.robot.commands.ArmCmd;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;


public class ArmDownCmd extends Command{
    
ArmSubsystem m_armSubsytem;
double setpoint;
    
    public ArmDownCmd(ArmSubsystem armSubsystem){ //,double setpoint){
    
          this.m_armSubsytem = armSubsystem;
          //this.setpoint = setpoint;
      addRequirements(m_armSubsytem);
    }

    @Override
  public void initialize() {
    
    

  }

  @Override
  public void execute() {
    
    /*double error = setpoint - m_armSubsytem.getEncoderArmValue();
    double speed = ( 0.01 * error) + 0.2*Math.sin(m_armSubsytem.getEncoderArmValue());
    
    
    if(speed <= -0.3){
      m_armSubsytem.setArmMotor(-0.3);;
    
    } else{
      m_armSubsytem.setArmMotor(speed);
    }*/
    //double speed = -0.3 * Math.sin(m_armSubsytem.getEncoderArmValue());
  m_armSubsytem.setArmMotor(-0.3);
  }

  @Override
  public boolean isFinished() {
    if(m_armSubsytem.getEncoderArmValue() <= 5){
      return true;
    }
    return false;
  }
  

  @Override
  public void end(boolean interrupted){
    m_armSubsytem.stopArm();
  }
}
