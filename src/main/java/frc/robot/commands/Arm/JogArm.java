// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ArmSubsystem;

public class JogArm extends Command {
  /** Creates a new JogArm. */
  private ArmSubsystem m_arm;
  private CommandXboxController m_controller;

  public JogArm(ArmSubsystem arm, CommandXboxController controller) {
    m_arm = arm;
    m_controller = controller;
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_arm.enableArm = false;
    m_arm.enableArm = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean allowDown = true;
    boolean allowUp = true;
    if (RobotBase.isReal()) {
      allowUp = m_arm.getAngle().gt(m_arm.minAngle)
          || m_controller.getHID().getBackButton();

      allowDown = m_arm.getAngle().lt(m_arm.maxAngle)
          || m_controller.getHID().getBackButton();
    }

    else {
      allowUp = m_arm.armMotor.getEncoder().getPosition()
         < (m_arm.maxAngle.in(Radians))
          || m_controller.getHID().getBackButton();

      allowDown = m_arm.armMotor.getEncoder().getPosition()
       > (m_arm.minAngle.in(Radians))
          || m_controller.getHID().getBackButton();

    }

    double yval = -m_controller.getLeftY() / 2;

    SmartDashboard.putNumber("Arm/VoltsJog", m_arm.appliedVolts);
  
   
    if (yval > 0 && allowUp || yval < 0 && allowDown) {

      m_arm.appliedVolts = yval * RobotController.getBatteryVoltage();

      m_arm.armMotor.setVoltage(m_arm.appliedVolts);

    } else {
      m_arm.armMotor.setVoltage(0);
    }

    m_arm.setTarget(m_arm.getAngle());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_arm.enableArm = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
