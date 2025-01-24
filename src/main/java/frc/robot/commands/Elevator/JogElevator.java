// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class JogElevator extends Command {
  /** Creates a new JogArm. */
  private final CommandXboxController gamepad;
  private final ElevatorSubsystem elevator;
  private final double deadband = .00;

  public JogElevator(ElevatorSubsystem elevator, CommandXboxController gamepad) {
      this.elevator = elevator;
      this.gamepad = gamepad;
      addRequirements(this.elevator);
  }

  @Override
  public void initialize() {
      //gamepad.rumble(250);
  }

  @Override
  public void execute() {

      double stickValue = -gamepad.getLeftY();

      if (Math.abs(stickValue) < deadband) stickValue = 0;

      double leftPower = stickValue / 2;
      double rightPower = stickValue / 2;

      boolean overrideLimits = gamepad.start().getAsBoolean();

      if (overrideLimits || (leftPower > 0 && elevator.getLeftPositionInches() < elevator.UPPER_POSITION_LIMIT
              || leftPower < 0 && elevator.getLeftPositionInches() > elevator.LOWER_POSITION_LIMIT)
              || (rightPower > 0 && elevator.getRightPositionInches() < elevator.UPPER_POSITION_LIMIT
              || rightPower < 0 && elevator.getRightPositionInches() > elevator.LOWER_POSITION_LIMIT)) {
          elevator.m_leftMotor.set(leftPower);
          elevator.m_rightMotor.set(leftPower);
      } else {
        elevator.m_leftMotor.set(0);
          elevator.m_rightMotor.set(0);
      }

      elevator.setTargetInches(elevator.getLeftPositionInches());
  }

  @Override
  public void end(boolean interrupted) {
      elevator.setTargetInches(elevator.getLeftPositionInches());
      elevator.m_leftMotor.set(0);
      elevator.m_rightMotor.set(0);
      // gamepad.rumble(250);
  }

  @Override
  public boolean isFinished() {
      return false;
  }
}
