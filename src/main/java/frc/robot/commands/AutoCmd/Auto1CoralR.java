package frc.robot.commands.AutoCmd;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveCmd.DriveForDistanceCmd;
import frc.robot.commands.DriveCmd.TurnToAngleCmd;

public class Auto1CoralR extends SequentialCommandGroup{
    public Auto1CoralR(){
        addCommands(new DriveForDistanceCmd(2.2));
        addCommands(new TurnToAngleCmd(RobotContainer.driveSubsystem,-30));
        addCommands(new DriveForDistanceCmd(0.4));
        addCommands(new DriveForDistanceCmd(-1));
        addCommands(new TurnToAngleCmd(RobotContainer.driveSubsystem,-100));
        addCommands(new DriveForDistanceCmd(-4.3));
        addCommands(new DriveForDistanceCmd(4.3));
        addCommands(new TurnToAngleCmd(RobotContainer.driveSubsystem,120));
        addCommands(new DriveForDistanceCmd(1.3));
    }
}
