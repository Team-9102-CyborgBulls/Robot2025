package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class Trajectory1 extends Command {
    private final DriveSubsystem driveSubsystem;

    
    public Trajectory1(DriveSubsystem drivesubsystem) {
        this.driveSubsystem = drivesubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivesubsystem);
  }

   public void generateTrajectory() {

    // 2018 cross scale auto waypoints.
    var sideStart = new Pose2d(0, 0,
        Rotation2d.fromDegrees(0));
    var crossScale = new Pose2d(1, 0,
        Rotation2d.fromDegrees(0));

    var interiorWaypoints = new ArrayList<Translation2d>();
    /*interiorWaypoints.add(new Translation2d(Units.feetToMeters(14.54), Units.feetToMeters(23.23)));
    interiorWaypoints.add(new Translation2d(Units.feetToMeters(21.04), Units.feetToMeters(18.23)));
    */
    TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(12), Units.feetToMeters(12));
    config.setReversed(true);

    var trajectory = TrajectoryGenerator.generateTrajectory(
        sideStart,
        interiorWaypoints,
        crossScale,
        config);
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
