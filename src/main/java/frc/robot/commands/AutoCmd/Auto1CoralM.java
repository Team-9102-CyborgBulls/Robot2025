package frc.robot.commands.AutoCmd;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveCmd.DriveForDistanceCmd;

public class Auto1CoralM extends SequentialCommandGroup{
    public Auto1CoralM(){
        addCommands(new DriveForDistanceCmd(1.93));
        
    }
}
