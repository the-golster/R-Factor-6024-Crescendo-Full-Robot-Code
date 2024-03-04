// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;

/**
 * An example command that uses an example subsystem.
 */
public class AutoIntake extends Command {

    private final SwerveSubsystem swerve;
    private final PIDController PID;

    public AutoIntake(SwerveSubsystem swerve) {
        this.swerve = swerve;
        this.PID = new PIDController(Constants.Arm.kp, 0, Constants.Arm.kd);

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        PID.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double TX = LimelightHelpers.getTX("");
        double TY = LimelightHelpers.getTY("");
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (Constants.smartEnable) {
            SmartDashboard.putString("Status", "finished");
        }
        return false;
    }

}
