// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class PositionHoldArm extends Command {
  /** Creates a new JogArm. */
  private ArmSubsystem m_arm;
  
    public PositionHoldArm(ArmSubsystem arm) {
      // Use addRequirements() here to declare subsystem dependencies.
      m_arm = arm;
  
      addRequirements(m_arm);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      SmartDashboard.putNumber("BegPos",m_arm.getAngle().in(Degrees));
      m_arm.armController.reset(m_arm.getAngle().in(Radians));
      m_arm.armController.setGoal(m_arm.getAngle().in(Radians));
  
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      m_arm.position();
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      m_arm.setGoal(Degrees.of(m_arm.getAngle().in(Degrees)));
  
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
  }