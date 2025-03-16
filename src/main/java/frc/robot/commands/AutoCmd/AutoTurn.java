package frc.robot.commands.AutoCmd;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveCmd.TurnToAngleCmd;
import frc.robot.commands.ElevatorCmd.ElevatorUpCmd;

public class AutoTurn extends SequentialCommandGroup{
    
    public AutoTurn(){
        addCommands(new TurnToAngleCmd(RobotContainer.driveSubsystem, 90));
        addCommands(new ElevatorUpCmd(RobotContainer.elevatorSubsystem));

    }
    
}
