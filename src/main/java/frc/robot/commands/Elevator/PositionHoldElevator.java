// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class PositionHoldElevator extends Command {
    private final ElevatorSubsystem elevator;
    private final ArmSubsystem m_arm;
    private Debouncer armClearDebouncer;

    public PositionHoldElevator(ElevatorSubsystem elevator, ArmSubsystem arm) {
        this.elevator = elevator;
        m_arm = arm;
        addRequirements(this.elevator);
    }

    @Override
    public void initialize() {
        armClearDebouncer = new Debouncer(.2, DebounceType.kRising);
        double temp = elevator.getLeftPositionMeters();
        elevator.setGoalMeters(temp);

    }

    @Override
    public void execute() {

        boolean openLoop = elevator.getGoalInches() < 1 && elevator.getLeftPositionInches() < 1;

        if (!openLoop)
            elevator.position();

        SmartDashboard.putNumber("Elevator/poslim",
                Units.radiansToDegrees(m_arm.armMotor.getEncoder().getPosition()));
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

  
}