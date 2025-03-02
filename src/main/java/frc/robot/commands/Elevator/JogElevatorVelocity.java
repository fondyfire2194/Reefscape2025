// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class JogElevatorVelocity extends Command {
    /** Creates a new JogArm. */
    private final CommandXboxController gamepad;
    private final ElevatorSubsystem elevator;
    private final double deadband = .01;
    private final double scaleFactor = 10;
    private double mps;

    public JogElevatorVelocity(ElevatorSubsystem elevator, CommandXboxController gamepad) {
        this.elevator = elevator;
        this.gamepad = gamepad;
        addRequirements(this.elevator);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        double stickValue = -gamepad.getRightY() / scaleFactor;

        if (Math.abs(stickValue) < deadband)
            stickValue = 0;

        mps = stickValue * elevator.maxVelocityMPS;

        boolean overrideLimits = gamepad.start().getAsBoolean();

        if (overrideLimits ||
                mps > 0 && !elevator.atUpperLimit
                || mps < 0 && !elevator.atLowerLimit) {
            elevator.runAtVelocity(mps);
        } else {
            elevator.runAtVelocity(0);
        }
        SmartDashboard.putNumber("Elevator/jog_mps", mps);

        elevator.pidGoalMeters = elevator.getLeftPositionMeters();
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setGoalMeters(elevator.leftMotor.getEncoder().getPosition());
        elevator.runAtVelocity(0);
        elevator.pidGoalMeters = elevator.getLeftPositionMeters();
        SmartDashboard.putNumber("Elevator/jog_mps", mps);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
