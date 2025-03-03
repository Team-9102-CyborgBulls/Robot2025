// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCmd;

import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class DriveCmd extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem driveSubsystem;
  SlewRateLimiter speedLimiter = new SlewRateLimiter(2); // Limite de 2 unités par seconde

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveCmd(DriveSubsystem drivesubsystem) {
    this.driveSubsystem = drivesubsystem;
    
    addRequirements(drivesubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSubsystem.resetEncoders();

   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      SmartDashboard.putNumber("speed changer value", driveSubsystem.speed_changer); // Affichage de la valeur de speed_changer sur le SmartDashboard

     
      //SmartDashboard.putNumber("direction value", driveSubsystem.direction); // Affichage de la valeur de direction sur le SmartDashboard
        
        double forwardSpeed = RobotContainer.manette.getLeftY(); // Récupération de la vitesse de déplacement vers l'avant
        double turnSpeed =  RobotContainer.manette.getRightX(); // Récupération de la vitesse de rotation

       
        double limiteSpeed = speedLimiter.calculate(forwardSpeed);
        //double targetSpeed = driveSubsystem.getTargetSpeed(forwardSpeed,0);

        

        driveSubsystem.arcadeDrive(-limiteSpeed, turnSpeed); // Appel de la méthode arcadeDrive du sous-système driveSubsystem avec les vitesses calculées
        
        

        if( forwardSpeed == 0 && turnSpeed == 0){ // Vérification si les vitesses sont nulles
            //driveSubsystem.arcadeDrive(0, 0); // Arrêt du mouvement si les vitesses sont nulles
        }

        SmartDashboard.putNumber("ForwardSpeed", forwardSpeed);
        SmartDashboard.putNumber("LimiteSpeed", limiteSpeed);
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
