// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ArmSubsystem;

public class PositionHoldArmPID extends Command {
    private final ArmSubsystem arm;

    private PIDController pidController;
    private double kp = 10.;
    private double ki = 0;
    private double kd;
    private double izone = .5;
    private double minIntegral = -.1;
    private double maxIntegral = .1;
    private double tolerance = Units.inchesToMeters(1);
    private double maxrate = 2;
    private boolean toggle;

    public PositionHoldArmPID(ArmSubsystem arm) {
        this.arm = arm;

        pidController = new PIDController(kp, ki, kd);
        addRequirements(this.arm);
    }

    @Override
    public void initialize() {
        pidController.setIZone(izone);
        pidController.disableContinuousInput();
        pidController.setIntegratorRange(minIntegral, maxIntegral);
        pidController.setTolerance(tolerance);
        double temp = arm.getAngleRadians();
        arm.setGoalRadians(temp);
        SmartDashboard.putData(" Arm/PID/controller", pidController);
    }

    @Override
    public void execute() {

        toggle = !toggle;

        arm.nextSetpoint = arm.m_profile.calculate(.02, arm.currentSetpoint, arm.m_goal);

        double radpersec = pidController.calculate(arm.getAngleRadians(), arm.nextSetpoint.position);

        if (toggle) {

            SmartDashboard.putNumber("Arm/PID/goalpos", arm.m_goal.position);
            SmartDashboard.putNumber("Arm/PID/currsetpos", arm.currentSetpoint.position);
            SmartDashboard.putNumber("Arm/PID/currsetvel", arm.currentSetpoint.velocity);
            SmartDashboard.putNumber("Arm/PID/setpos", arm.nextSetpoint.position);
        } else {
            SmartDashboard.putNumber("Arm/PID/setvel", arm.nextSetpoint.velocity);
            SmartDashboard.putNumber("Arm/PID/radpersec", radpersec);
            SmartDashboard.putNumber("Arm/PID/poserror", pidController.getError());
            SmartDashboard.putBoolean("Arm/PID/poserror", pidController.atSetpoint());
        }
        radpersec = MathUtil.clamp(radpersec, -maxrate, maxrate);

        SmartDashboard.putNumber("Arm/PID/mpsclamped", radpersec);

        arm.runAtVelocity(radpersec);

        arm.currentSetpoint = arm.nextSetpoint;
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}