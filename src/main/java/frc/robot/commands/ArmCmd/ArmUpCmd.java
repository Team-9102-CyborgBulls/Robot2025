package frc.robot.commands.ArmCmd;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;


public class ArmUpCmd extends Command{
    
ArmSubsystem m_armSubsytem;
double setpoint;
    
    public ArmUpCmd(ArmSubsystem armSubsystem ){//,double setpoint){
    
          this.m_armSubsytem = armSubsystem;
          //this.setpoint = setpoint;
        addRequirements(m_armSubsytem);
    }

    @Override
  public void initialize() {
    


  }

  @Override
  public void execute() {
    
    //double error = setpoint - m_armSubsytem.getEncoderArmValue();
    //double speed = ( 0.01 * error) + 0.2*Math.sin(m_armSubsytem.getEncoderArmValue());
    
    
    /*if(speed >= 0.3){
      m_armSubsytem.setArmMotor(0.3);;
    
    } else{
      m_armSubsytem.setArmMotor(speed);
    }*/
    //m_armSubsytem.setArmMotor(0.2);
  }

  @Override
  public boolean isFinished() {
    
    
    return false;
  }
  

  @Override
  public void end(boolean interrupted){
    m_armSubsytem.stopArm();
  }
}
