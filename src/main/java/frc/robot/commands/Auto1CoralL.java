package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class Auto1CoralL extends SequentialCommandGroup{
    public Auto1CoralL(){
        addCommands(new DriveForDistanceCmd(2.46));
        addCommands(new TurnToAngleCmd(RobotContainer.driveSubsystem,30));
        addCommands(new DriveForDistanceCmd(0.3));
        //addCommands(new DriveForDistanceCmd(-0.3));
        //addCommands(new TurnToAngleCmd(RobotContainer.driveSubsystem,-60));
    }
}
