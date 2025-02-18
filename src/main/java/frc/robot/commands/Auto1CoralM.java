package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Auto1CoralM extends SequentialCommandGroup{
    public Auto1CoralM(){
        addCommands(new DriveForDistanceCmd(1.93));
        
    }
}
