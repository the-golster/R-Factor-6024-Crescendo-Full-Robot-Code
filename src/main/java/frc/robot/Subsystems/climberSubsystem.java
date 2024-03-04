// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants;

public class climberSubsystem extends SubsystemBase {
    
    final CANSparkMax motor = new CANSparkMax(Constants.Climber.climberID, MotorType.kBrushless);
    final RelativeEncoder encoder = motor.getEncoder();
    /** Creates a new Climber. */
    
    public climberSubsystem() {
      motor.setInverted(Constants.Climber.climberInvert);      
    }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setMotor(double speed) {
    motor.set(0);
    motor.set(speed);
  }

  public double getLeftEncoder(){
    return encoder.getPosition();
  }
}
