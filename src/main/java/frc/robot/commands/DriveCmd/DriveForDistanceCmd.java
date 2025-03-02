package frc.robot.commands.DriveCmd;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

// This command drives a specified number of meters
public class DriveForDistanceCmd extends Command {

  DriveSubsystem m_DriveSubsystem;
  double initialDistance;
  double targetDistance;
  double distance;
  double percentPower;
  double kP;
  double error;
  public static boolean finCmd = false;

  /** Creates a new DriveForDistanceCommand. */
  public DriveForDistanceCmd(double distance) {
    m_DriveSubsystem = RobotContainer.driveSubsystem;
    this.distance = distance;
    //this.percentPower = percentPower;
    addRequirements(m_DriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    m_DriveSubsystem.resetEncoders();
    initialDistance = -m_DriveSubsystem.getAverageEncoderDistance();
    System.out.println("INITIAL DISTANCE: " + initialDistance);
    this.targetDistance = initialDistance + distance;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double kP = 1;
    double currentPosition = -m_DriveSubsystem.getAverageEncoderDistance();
    error = targetDistance - currentPosition;
    double speed =  kP * error;

    if(speed >= 0.5){

      m_DriveSubsystem.setDriveMotors(0.5, 0);
    }else if (speed <= -0.5){
      m_DriveSubsystem.setDriveMotors(-0.5, 0);
    }
    else{
    m_DriveSubsystem.setDriveMotors(speed, 0);
    }
    // Print statements for debugging
    System.out.println("GOAL DISTANCE: " + (distance + initialDistance));
    System.out.println("CURRENT DISTANCE: " + -m_DriveSubsystem.getAverageEncoderDistance());
    System.out.println("POWER: " + percentPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveSubsystem.stop(); // Stop the drivetrain motors
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    if(error >= 0 && error <= 0.3){
      finCmd = true;
      return true;
      
      
    }
    else if(error <= 0 && error >= -0.3){
      finCmd = true;
      return true;
      
    }
    else{
      return false;
    }
    
    
    
    
    /*if(distance >= 0){
      return m_DriveSubsystem.getDistance() >= initialDistance + distance; // End the command when we have reached our goal
    }else if(distance <= 0){
      return m_DriveSubsystem.getDistance() <= initialDistance + distance; // End the command when we have reached our goal
    }else{
      return false;
    }*/
  }
}
