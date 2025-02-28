package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Servo;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  
  SparkMax m_armMotor = new SparkMax(7, MotorType.kBrushless);
  SparkMax m_intakeArm = new SparkMax(6, MotorType.kBrushless);

  SparkMaxConfig configArm = new SparkMaxConfig();
  
  public ArmSubsystem() {
        configArm.voltageCompensation(12);
        configArm.smartCurrentLimit(40);
        configArm.idleMode(IdleMode.kCoast);

        //m_armMotor.getEncoder().setPosition(0);

  }

  public void setArmMotor(double speed) {
    m_armMotor.set(speed);
  }

  public void stopArm() {
    m_armMotor.set(0);
  }

  public void setIntakeArm(){
    m_intakeArm.set(0.2);
  }

  public void setIntakeArmReverse(){
    m_intakeArm.set(-0.2);
  }

  public void stopIntakeArm(){
    m_intakeArm.set(0);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("angle servo",intakeservo.getAngle());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}