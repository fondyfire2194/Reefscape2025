// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class PositionHoldElevator extends Command {
    private final ElevatorSubsystem elevator;
    private final ArmSubsystem m_arm;

    public PositionHoldElevator(ElevatorSubsystem elevator, ArmSubsystem arm) {
        this.elevator = elevator;
        m_arm = arm;
        addRequirements(this.elevator);
    }

    @Override
    public void initialize() {
        elevator.posrng = 911;
        double temp = elevator.getLeftPositionMeters();
        elevator.setGoalMeters(temp);
    }

    @Override
    public void execute() {
        elevator.position();

        elevator.armAngle = m_arm.armMotor.getEncoder().getPosition();
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}