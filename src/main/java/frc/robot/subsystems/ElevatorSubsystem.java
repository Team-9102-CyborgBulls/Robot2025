
package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    
    SparkMax m_elevatorMotor = new SparkMax(5, MotorType.kBrushless);
    static SparkMaxConfig configElevator = new SparkMaxConfig();

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
}
