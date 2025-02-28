package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  
  
  public Servo intakeservo = new Servo(9);
  public IntakeSubsystem() {

  }
/*Example command factory method.*
@return a command*/
public Command ServoDefaultCmd(IntakeSubsystem intakeSubsystem, double angle) {// Inline construction of command goes here.// Subsystem::RunOnce implicitly requires this subsystem.
  return Commands.run(() -> intakeservo.setAngle(angle), intakeSubsystem);

}

  public void setServo(double pos) {
    intakeservo.set(pos);
  }

  public void setIntakeZero() {
    intakeservo.setAngle(0);
  }
  /**
   
An example method querying a boolean state of the subsystem (for example, a digital sensor).*
@return value of some boolean subsystem state, such as a digital sensor.*/
public boolean exampleCondition() {// Query some boolean state, such as a digital sensor.
  return false;
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