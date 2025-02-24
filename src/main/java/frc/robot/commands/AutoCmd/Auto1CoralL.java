package frc.robot.commands.AutoCmd;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveCmd.DriveForDistanceCmd;
import frc.robot.commands.DriveCmd.TurnToAngleCmd;

public class Auto1CoralL extends SequentialCommandGroup{
    public Auto1CoralL(){
        addCommands(new DriveForDistanceCmd(2.46));
        addCommands(new TurnToAngleCmd(RobotContainer.driveSubsystem,30));
        addCommands(new DriveForDistanceCmd(0.3));
        //addCommands(new DriveForDistanceCmd(-0.3));
        //addCommands(new TurnToAngleCmd(RobotContainer.driveSubsystem,-60));
    }
}
