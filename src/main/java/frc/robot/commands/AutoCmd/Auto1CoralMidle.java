package frc.robot.commands.AutoCmd;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveCmd.DriveForDistanceCmd;
import frc.robot.commands.ElevatorCmd.ElevatorDownCmd;
import frc.robot.commands.ElevatorCmd.ElevatorUpCmd;
import frc.robot.commands.OutTakeCmd.OutTakeAutoCmd;
import frc.robot.subsystems.ElevatorSubsystem;

public class Auto1CoralMidle extends SequentialCommandGroup{
    public Auto1CoralMidle(){
        addCommands(new DriveForDistanceCmd(2.35));
        addCommands(new ElevatorUpCmd(RobotContainer.elevatorSubsystem));
        addCommands(new OutTakeAutoCmd(RobotContainer.outTakeSubsystem));
        addCommands(new ElevatorDownCmd(RobotContainer.elevatorSubsystem));
        
    }
}
