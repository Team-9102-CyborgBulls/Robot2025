package frc.robot.commands.ArmCmd;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ArmSubsystem;

public class ArmStillCmd extends Command{
    
ArmSubsystem m_armSubsytem;
    
    public ArmStillCmd(ArmSubsystem armSubsystem){
    
          this.m_armSubsytem = armSubsystem;
      addRequirements(m_armSubsytem);
    }

    @Override
  public void initialize() {
    
    
  }

  @Override
  public void execute() {
    m_armSubsytem.setArmMotor(-0.02);
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
