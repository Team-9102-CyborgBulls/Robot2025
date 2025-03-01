
package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    
    SparkMax m_elevatorMotor = new SparkMax(Constants.CANIdConstants.ID_MOTEUR_ELEVATOR, MotorType.kBrushless);
    static SparkMaxConfig configElevator = new SparkMaxConfig();
    
    public RelativeEncoder elevatorEncoder = m_elevatorMotor.getEncoder();

    public DutyCycleEncoder cypher = new DutyCycleEncoder(9);

    public ElevatorSubsystem(){

        configElevator.voltageCompensation(12);
        configElevator.smartCurrentLimit(40);
        configElevator.idleMode(IdleMode.kCoast);

    
    }

    public void setElevatorMotor(double speed){
        m_elevatorMotor.set(-speed);
    }

    public void stop(){
        m_elevatorMotor.set(0);
    }

    public double getEncoderValue(){

        double cpt = 0;

        if (cypher.get() >= 1){
            cpt+=1;
        }

        double value = cpt + cypher.get();
        
        return value;
    }
    public double changesetpoint(double elevation){
        if(elevation == Constants.ElevatorConstants.ELEVATOR_L2_POSITION){
            elevation = Constants.ElevatorConstants.ELEVATOR_L3_POSITION;
        }

        else if(elevation == Constants.ElevatorConstants.ELEVATOR_L3_POSITION){
            elevation = Constants.ElevatorConstants.ELEVATOR_L2_POSITION;
        }
        return elevation;
    }
    
}
