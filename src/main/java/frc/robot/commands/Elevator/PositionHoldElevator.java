// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class PositionHoldElevator extends Command {
  private final ElevatorSubsystem elevator;

  public PositionHoldElevator(ElevatorSubsystem elevator) {
      this.elevator = elevator;
      addRequirements(this.elevator);
  }

  @Override
  public void initialize() {
      elevator.posrng = 911;
      double temp = elevator.getLeftPositionInches();
      elevator.setTargetInches(temp);
  }

  @Override
  public void execute() {

      elevator.position();
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
      return false;
  }
}