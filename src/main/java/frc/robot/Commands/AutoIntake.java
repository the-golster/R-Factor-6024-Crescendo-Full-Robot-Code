// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.armSubystem;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;

/**
 * An example command that uses an example subsystem.
 */
public class AutoIntake extends Command {

    private final SwerveSubsystem swerve;
    private final PIDController PIDang;
    private final PIDController PIDtrans;

    public AutoIntake(SwerveSubsystem swerve) {
        this.swerve = swerve;
        this.PIDang = new PIDController(1, 0, 0);
        this.PIDtrans = new PIDController(1, 0, 0);

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        PIDang.reset();
        PIDtrans.reset();        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double TX = Math.toRadians(LimelightHelpers.getTX(""));
        double TY = Math.toRadians(LimelightHelpers.getTY(""));

        PIDang.setSetpoint(0);
        double rot = PIDang.calculate(TX);

        double ll_height = 0.1;

        // tan(TY) = x/ll_height
        // x = tan(TY) * ll_height
        double x = Math.tan(TY) * ll_height;

        PIDtrans.setSetpoint(0);
        PIDtrans.calculate(x);

        swerve.drive(new Translation2d(x, 0), rot, false);        
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
