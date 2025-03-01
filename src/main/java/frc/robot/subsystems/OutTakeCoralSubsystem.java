package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class OutTakeCoralSubsystem extends SubsystemBase {
    
    SparkMax m_outTakeMotor = new SparkMax(Constants.CANIdConstants.ID_MOTEUR_OUTTAKE_CORAL, MotorType.kBrushless);
    static SparkMaxConfig configOut = new SparkMaxConfig();
    public DigitalInput intake_sensor = new DigitalInput(8);
    

    public OutTakeCoralSubsystem(){

        configOut.voltageCompensation(12);
        configOut.smartCurrentLimit(40);
        configOut.idleMode(IdleMode.kCoast);
    }

    public void setOutTakeMotor(double speed){
        m_outTakeMotor.set(speed);
    }

    public void stop(){
        m_outTakeMotor.set(0);
    }
}
