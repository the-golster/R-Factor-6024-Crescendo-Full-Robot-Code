// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.armSubystem;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;

/**
 * An example command that uses an example subsystem.
 */
public class armPID extends Command {

    private final armSubystem Arm;
    private final PIDController PID;
    private final double target_angle;

    public armPID(armSubystem Arm, double target_angle) {
        this.Arm = Arm;
        this.PID = new PIDController(Constants.Arm.kp, 0, Constants.Arm.kd);
        this.target_angle = target_angle;

        addRequirements(Arm);
    }

    @Override
    public void initialize() {
        PID.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double angle = this.target_angle;

            

        double absoluteEncoder = 2 * Arm.getAbsoluteEncoder() * Math.PI;

        double speed = PID.calculate(absoluteEncoder, angle);

        Arm.setMotors(speed, speed);

        if (Constants.smartEnable) {
            SmartDashboard.putNumber("AngleS", angle);
            SmartDashboard.putNumber("speed", speed);
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}
