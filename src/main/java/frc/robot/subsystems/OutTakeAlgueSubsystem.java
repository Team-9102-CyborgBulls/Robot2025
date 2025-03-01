// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class OutTakeAlgueSubsystem extends SubsystemBase {

  WPI_TalonSRX m_OutTakeAlgueMotor = new WPI_TalonSRX(Constants.CANIdConstants.ID_MOTEUR_OUTTAKE_ALGUE);
  DigitalInput limitSwitch = new DigitalInput(7);
  
  public OutTakeAlgueSubsystem() {

    
  }
  public void setOutTakeAlgueMotor(double speed){
    m_OutTakeAlgueMotor.set(speed);
  }
  public boolean limitSwitchActivated(){
    return limitSwitch.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
