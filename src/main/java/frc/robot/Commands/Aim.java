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
public class Aim extends Command {

    private final SwerveSubsystem swerve;
    private final armSubystem Arm;
    private final PIDController PID;

    public Aim(SwerveSubsystem swerve, armSubystem Arm) {
        this.swerve = swerve;
        this.Arm = Arm;
        this.PID = new PIDController(Constants.Arm.kp, 0, Constants.Arm.kd);

        addRequirements(swerve, Arm);
    }

    @Override
    public void initialize() {
        PID.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double ID = LimelightHelpers.getFiducialID("");
        double TX = LimelightHelpers.getTX("");
        double TY = LimelightHelpers.getTY("");
        final Translation2d noTrans = new Translation2d(0, 0);

        double angleS, h1, h2, d1, d2, d3, h, d, r;

        if (ID == 7 || ID == 4 || ID == 5 || ID == 4) {
            if (ID == 7 || ID == 4) {

                h1 = 1;
                h2 = 1;
                d2 = 1;
            } else {

                h1 = 1;
                h2 = 1;
                d2 = 1;
            }

            h = h1 + h2;
            d1 = h1 / Math.tan(TY * Math.PI / 180);
            d = d1 + d2;
            angleS = Math.atan(h / d);

            d3 = d2 / 2;
            r = Math.atan((d1 * Math.tan(TX * Math.PI / 180)) / (d1 + d3));

            swerve.drive(noTrans, r, false);

            double absoluteEncoder = 2 * Arm.getAbsoluteEncoder() * Math.PI;

            double speed = PID.calculate(absoluteEncoder, angleS);

            Arm.setMotors(speed, speed);

            if (Constants.smartEnable) {
                SmartDashboard.putNumber("AngleS", angleS);
                SmartDashboard.putNumber("Speed", speed);
                SmartDashboard.putNumber("d1", d1);
                SmartDashboard.putString("Status", "aiming");
            }

        }
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
