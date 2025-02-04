// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Coral;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DetectCoralWhileIntaking extends Command {

  private final CoralIntakeSubsystem m_coral;

  private MedianFilter sampleFilter;
  private MedianFilter detectFilter;
  private int coralDetectLevel = 20;
  private int sampleFilterLevel = 5;
  private int sampleCount;
  private final int numberSamplesWanted = 50;// 1 second
  private int detectCount;
  private final int numberDetectsWanted = 25;// 1 second
  private double filteredRPM;
  private boolean coralDetected = false;

  public DetectCoralWhileIntaking(CoralIntakeSubsystem coral) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_coral = coral;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sampleFilter = new MedianFilter(sampleFilterLevel);
    detectFilter = new MedianFilter(coralDetectLevel);
    sampleCount = 0;
    detectCount = 0;
    coralDetected = false;
    sampleFilter.reset();
    detectFilter.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    sampleCount++;
    if (sampleCount <= numberSamplesWanted)
      filteredRPM = sampleFilter.calculate(m_coral.getRPM());
    else
      filteredRPM = detectFilter.calculate(m_coral.getRPM());

    if (sampleCount > numberSamplesWanted && detectCount > numberDetectsWanted) {
      detectCount++;
      coralDetected = detectCount > numberDetectsWanted && filteredRPM < filteredRPM;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_coral.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return coralDetected;
  }
}
