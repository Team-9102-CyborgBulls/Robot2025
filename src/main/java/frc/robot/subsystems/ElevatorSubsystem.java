
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
import edu.wpi.first.math.MathUtil;

public class ElevatorSubsystem extends SubsystemBase {
    
    SparkMax m_elevatorMotor = new SparkMax(Constants.CANIdConstants.ID_MOTEUR_ELEVATOR, MotorType.kBrushless);
    static SparkMaxConfig configElevator = new SparkMaxConfig();
    
    public RelativeEncoder elevatorEncoder = m_elevatorMotor.getEncoder();

    public  double setelevator = Constants.ElevatorConstants.ELEVATOR_L2_POSITION;

    

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
        return (-elevatorEncoder.getPosition()/25)*(2*Math.PI*2.45);
      }
  
      public void resetElevatorEncoder(){
          elevatorEncoder.setPosition(0);
      }

    public double changesetpoint(double elevation){
        if(elevation == Constants.ElevatorConstants.ELEVATOR_L2_POSITION){
            elevation = Constants.ElevatorConstants.ELEVATOR_L3_POSITION;
        }

        else if(elevation == Constants.ElevatorConstants.ELEVATOR_L3_POSITION){
            elevation = Constants.ElevatorConstants.ELEVATOR_L2_POSITION;
        }

        else if(elevation == Constants.ElevatorConstants.ELEVATOR_L1_POSITION){
            elevation = Constants.ElevatorConstants.ELEVATOR_L2_POSITION;
        }
        return elevation;
    }
    
}
