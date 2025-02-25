package frc.robot.commands;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class GettingInRangeCmd extends Command {
      // Vision-alignment mode
        // Query the latest result from PhotonVision
    
    DriveSubsystem driveSubsystem;
    VisionSubsystem visionSubsystem;
    double range;
  
    public GettingInRangeCmd(DriveSubsystem drivesubsystem, VisionSubsystem visionSubsystem){

        this.driveSubsystem = drivesubsystem;
        this.visionSubsystem = visionSubsystem;
       
        addRequirements(driveSubsystem);

    }

    @Override
  public void initialize() {
  }

  @Override
  public void execute() {
     
    double forwardSpeed = 0;
     double turnSpeed = 0;
    PhotonPipelineResult result = visionSubsystem.camera.getLatestResult();

        if (result.hasTargets()) {
            // First calculate range
            range =
                    PhotonUtils.calculateDistanceToTargetMeters(
                            VisionSubsystem.CAMERA_HEIGHT_METERS,
                            VisionSubsystem.TARGET_HEIGHT_METERS,
                            VisionSubsystem.CAMERA_PITCH_RADIANS,
                            Units.degreesToRadians(result.getBestTarget().getPitch()));

            // Use this range as the measurement we give to the PID controller.
            // -1.0 required to ensure positive PID controller effort _increases_ range
            forwardSpeed = -VisionSubsystem.controller.calculate(range, VisionSubsystem.GOAL_RANGE_METERS);
        } else if(result.hasTargets() == false) {
            // If we have no targets, stay still.
            forwardSpeed = 0;
        }
        if(forwardSpeed > 0.5){
          forwardSpeed = 0.5;
       }
       else if(forwardSpeed < -0.5){
         forwardSpeed = -0.5;
       }
    
        driveSubsystem.arcadeDrive(forwardSpeed, 0);
    
  }

  @Override
  public void end(boolean interrupted){
    System.out.println("a");
    driveSubsystem.setDriveMotors(0, 0);
  }
  

  @Override
  public boolean isFinished() {
    if(range > 2.42 && range < 3.02){
      System.out.println("c");
      return true;
    }
    else{
      return false;
    }
    
}
}