// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import frc.robot.Subsystems.climberSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example Climber. */
public class climberCmd extends Command {
  private final climberSubsystem climber;
  private final double speed;

  public climberCmd(climberSubsystem climber, double speed) {
    this.climber = climber;
    this.speed = speed;

    addRequirements(climber);
  }


  @Override
  public void initialize() {}


  @Override
  public void execute() {
    climber.setMotor(speed);
  }


  @Override
  public void end(boolean interrupted) {
    climber.setMotor(0);
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}
