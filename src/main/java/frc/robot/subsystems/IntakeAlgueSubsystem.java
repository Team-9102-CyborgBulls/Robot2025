package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeAlgueSubsystem extends SubsystemBase {
  
  
  public SparkMax m_intakeAlgue = new SparkMax(Constants.CANIdConstants.ID_MOTEUR_INTAKE_ALGUE, MotorType.kBrushless);
  public IntakeAlgueSubsystem() {

  }

  public void setIntakeAlgueMotor(double speed) {
    m_intakeAlgue.set(speed);
  }

  public void stopIntakeAlgue() {
    m_intakeAlgue.set(0);
  }
}


  
 