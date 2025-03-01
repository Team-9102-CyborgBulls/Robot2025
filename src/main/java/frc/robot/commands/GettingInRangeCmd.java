package frc.robot.commands;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveCmd.DriveCmd;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

//faut changer mais c'est une base
public class GettingInRangeCmd extends Command {
      // Vision-alignment mode
        // Query the latest result from PhotonVision
    
    DriveSubsystem driveSubsystem;
    VisionSubsystem visionSubsystem;
    RobotContainer m_robotContainer;
    public double range;

    double forwardSpeed = 0;
    double turnSpeed = 0;
    // Read in relevant data from the Camera
    boolean targetVisible = false;
    double targetYaw = 0.0;
    double targetRange = 0.0;

    double targetRange2;
  
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
     
    
    var results = visionSubsystem.photonCamera.getAllUnreadResults();
    if (!results.isEmpty()) {
        // Camera processed a new frame since last
        // Get the last one in the list.
        var result = results.get(results.size() - 1);
        if (result.hasTargets()) {
            // At least one AprilTag was seen by the camera
            for (var target : result.getTargets()) {
                if (target.getFiducialId() == 10) {
                    // Found Tag 10, record its information
                    targetYaw = target.getYaw();
                    targetRange =
                            PhotonUtils.calculateDistanceToTargetMeters(
                                     0.32, // Measured with a tape measure, or in CAD.
                                     0.17, // 
                                    Units.degreesToRadians(0), // Measured with a protractor, or in CAD.
                                    Units.degreesToRadians(target.getPitch()));
                    targetRange2 = target.getBestCameraToTarget().getX();
                    targetVisible = true;
                }
            }
        }
        
    }
    SmartDashboard.putNumber("TargetYaw", targetYaw);
    SmartDashboard.putNumber("ForwardSpeed",forwardSpeed);
    SmartDashboard.putNumber("turnSpeed", turnSpeed);
    SmartDashboard.putNumber("targetRange",targetRange);
    SmartDashboard.putNumber("targetRange2",targetRange2);

    if (targetVisible){
      
      turnSpeed = (0 - targetYaw) * 0.04;
      forwardSpeed = - (1.0 - targetRange2) * 0.4;
      
      
    }
    if(turnSpeed > 0.3){
      turnSpeed = 0.3;
    }
    else if(turnSpeed < -0.3){
    turnSpeed = -0.3;
    }
    if(forwardSpeed > 0.3){
      forwardSpeed = 0.3;
    }
    else if(forwardSpeed < -0.3){
    forwardSpeed = -0.3;
    }
    driveSubsystem.setDriveMotors(forwardSpeed, 0);

  }
  

  @Override
  public void end(boolean interrupted){
    System.out.println("a");
    driveSubsystem.setDriveMotors(0, 0);
  }
  

  @Override
  public boolean isFinished() {
    /*
    if(range > 0.8 && range < 1.2){
      System.out.println("c");
      return true;
    }
    else{
      return false;
    }
    */
    
    return false;
}
}