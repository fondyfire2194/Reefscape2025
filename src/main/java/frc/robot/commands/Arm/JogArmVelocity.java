// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ArmSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class JogArmVelocity extends Command {
    /** Creates a new JogArm. */
    private final CommandXboxController gamepad;
    private final ArmSubsystem arm;
    private final double deadband = .01;
    private final double scaleFactor = 10;
    private double radpersec;

    public JogArmVelocity(ArmSubsystem arm, CommandXboxController gamepad) {
        this.arm = arm;
        this.gamepad = gamepad;
        addRequirements(this.arm);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        double stickValue = -gamepad.getRightY() / scaleFactor;

        if (Math.abs(stickValue) < deadband)
            stickValue = 0;

        radpersec = stickValue * arm.maxradpersec;

        boolean overrideLimits = gamepad.start().getAsBoolean();

        if (overrideLimits ||
                radpersec > 0 && !arm.atUpperLimit
                || radpersec < 0 && !arm.atLowerLimit) {
            arm.runAtVelocity(radpersec);
        } else {
            arm.runAtVelocity(0);
        }
        SmartDashboard.putNumber("Arm/jog_radpersec", radpersec);
        SmartDashboard.putNumber("Arm/actangle", arm.getAngleRadians());
        SmartDashboard.putNumber("Arm/actvel", arm.getRadsPerSec());
        
     
        arm.setGoalRadians(arm.getAngleRadians());
    }

    @Override
    public void end(boolean interrupted) {
        arm.setGoalRadians(arm.getAngleRadians());
        arm.runAtVelocity(0);    
        SmartDashboard.putNumber("Arm/jog_radpersec", radpersec);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
