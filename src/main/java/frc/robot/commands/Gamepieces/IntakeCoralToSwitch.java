// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Gamepieces;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Factories.CommandFactory.ArmSetpoints;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GamepieceSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

public class IntakeCoralToSwitch extends Command {
  /** Creates a new IntakeCoralToswitch. */
  private final GamepieceSubsystem m_gamepiece;
  private final ArmSubsystem m_arm;

  public IntakeCoralToSwitch(GamepieceSubsystem gamepiece, ArmSubsystem arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_gamepiece = gamepiece;
    m_arm = arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_gamepiece.enableLimitSwitch();
    m_gamepiece.setCurrentLimit(m_gamepiece.inOutCoralAmps);
    m_gamepiece.gamepieceMotor.set(.25);
    // runGamepieceMotorAtVelocity(CoralRPMSetpoints.kGmepieceCoralIntakeRPM);
    m_gamepiece.coralIntakeMotor.set(5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.setGoalDegrees(ArmSetpoints.kTravel);
    m_gamepiece.stopCoralIntakeMotor();
    m_gamepiece.stopGamepieceMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_gamepiece.coralAtIntake();

  }
}
