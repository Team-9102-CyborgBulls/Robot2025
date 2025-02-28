package frc.robot.commands.ArmCmd;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ArmDownCmd extends Command{
    
ArmSubsystem m_armSubsytem;
    
    public ArmDownCmd(ArmSubsystem armSubsystem){
    
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
    m_armSubsytem.setArmMotor(0.2);
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
