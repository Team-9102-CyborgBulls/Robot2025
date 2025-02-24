package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class NothingCmd extends Command{

    private DriveSubsystem driveSubsystem;
    
        public NothingCmd(DriveSubsystem drivesubsystem) {
        this.driveSubsystem = drivesubsystem;
    
    addRequirements(drivesubsystem);
  }

  @Override
  public void execute() {
    driveSubsystem.setDriveMotors(0, 0);
  }
 // Called once the command ends or is interrupted.
 @Override
 public void end(boolean interrupted) {}

 // Returns true when the command should end.
 @Override
 public boolean isFinished() {
   return false;
 }
}
