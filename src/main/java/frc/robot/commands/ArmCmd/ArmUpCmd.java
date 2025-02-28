package frc.robot.commands.ArmCmd;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;


public class ArmUpCmd extends Command{
    
ArmSubsystem m_armSubsytem;
    
    public ArmUpCmd(ArmSubsystem armSubsystem){
    
          this.m_armSubsytem = armSubsystem;
      addRequirements(m_armSubsytem);
    }

    @Override
  public void initialize() {
    


  }

  @Override
  public void execute() {
    m_armSubsytem.setArmMotor(-0.2);
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
