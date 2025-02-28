package frc.robot.commands.ArmCmd;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;

public class IntakeArmReverseCmd extends Command{
    
ArmSubsystem m_armSubsytem;
    
    public IntakeArmReverseCmd(ArmSubsystem armSubsystem){
    
          this.m_armSubsytem = armSubsystem;
      addRequirements(m_armSubsytem);
    }

    @Override
  public void initialize() {
    
    RobotContainer.m_timer.reset();
    RobotContainer.m_timer.start();

  }

  @Override
  public void execute() {
    m_armSubsytem.setIntakeArmReverse();
  }

  @Override
  public boolean isFinished() {
    if(RobotContainer.m_timer.get() >= 2){
        return true ;
    } 
    return false;
  }
  

  @Override
  public void end(boolean interrupted){
    m_armSubsytem.stopIntakeArm();
  }
}
