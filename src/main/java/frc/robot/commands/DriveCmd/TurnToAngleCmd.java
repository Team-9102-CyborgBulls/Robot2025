package frc.robot.commands.DriveCmd;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToAngleCmd extends Command{
    
    DriveSubsystem driveSubsystem;
    double error;
    double targetAngle;
    double degreesToTurn;

    public TurnToAngleCmd(DriveSubsystem drivesubsystem, double degreesToTurn){

        this.driveSubsystem = drivesubsystem;
        this.degreesToTurn = degreesToTurn;
        
        addRequirements(driveSubsystem);

    }

    @Override
    public void initialize() {
       DriveSubsystem.gyro.reset();
       
        this.targetAngle = degreesToTurn +  DriveSubsystem.gyro.getAngle();
        SmartDashboard.putNumber("TargetAngle", targetAngle);
       
    }
  
    @Override
    public void execute() {
    
    double kP = 0.014;

    double currentPosition = DriveSubsystem.gyro.getAngle();
        // Find the heading error; setpoint is 90
    error = targetAngle - currentPosition;
    double speed = kP * error;

    // Turns the robot to face the desired direction
    if(speed >= 0.5){
        driveSubsystem.setDriveMotors(0,-0.5);
    }else if(speed <= -0.5){
         driveSubsystem.setDriveMotors(0,0.5);
    }else{
        
        driveSubsystem.setDriveMotors(0,-speed);

    }  
    SmartDashboard.putNumber("Position gyro",currentPosition);
    SmartDashboard.putNumber("Erreur gyro",error);

    }

    @Override
    public void end(boolean interrupted){
       
        driveSubsystem.setDriveMotors(0,0);

    }
    
  
    @Override
    public boolean isFinished() {
    
    if(Math.abs(error) <= 3){    
      return true;
    }
    else{
        return false;
        
    }
    }

}