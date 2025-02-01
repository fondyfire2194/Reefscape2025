// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Factories;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/** Add your docs here. */
public class CommandFactory {

        SwerveSubsystem m_swerve;
        ElevatorSubsystem m_elevator;
        ArmSubsystem m_arm;
        CoralIntakeSubsystem m_coral;
        AlgaeIntakeSubsystem m_algae;

        public CommandFactory(SwerveSubsystem swerve, ElevatorSubsystem elevator, ArmSubsystem arm,
                        CoralIntakeSubsystem coral, AlgaeIntakeSubsystem algae) {
                m_swerve = swerve;
                m_algae = algae;
                m_arm = arm;
                m_elevator = elevator;
                m_coral = coral;

        }

        public Command rumble(CommandXboxController controller, RumbleType type, double timeout) {
                return Commands.sequence(
                                Commands.race(
                                                Commands.either(
                                                                Commands.run(() -> controller.getHID().setRumble(type,
                                                                                1.0)),
                                                                Commands.runOnce(() -> SmartDashboard.putString("BUZZ",
                                                                                "BUZZ")),
                                                                () -> RobotBase.isReal()),
                                                Commands.waitSeconds(timeout)),
                                Commands.runOnce(() -> controller.getHID().setRumble(RumbleType.kBothRumble, 0.0)));
        }

        public boolean isRedAlliance() {
                Optional<Alliance> alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                        return alliance.get() == Alliance.Red;
                } else {
                        return false;
                }
        }

        public boolean isBlueAlliance() {
                Optional<Alliance> alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                        return alliance.get() == Alliance.Blue;
                } else {
                        return false;
                }
        }

        public enum Setpoint {
                kFeederStation,
                kLevel1,
                kLevel2,
                kLevel3,
                kLevel4;
        }

        public static final class ElevatorSetpoints {
                public static final int kFeederStation = 15;
                public static final int kLevel1 = 25;
                public static final int kLevel2 = 30;
                public static final int kLevel3 = 40;
                public static final int kLevel4 = 50;
        
        }

        public static final class ArmSetpoints {
                public static final double kFeederStation = 5;
                public static final double kLevel1 = 80;
                public static final double kLevel2 = 80;
                public static final double kLevel3 = 80;
                public static final double kLevel4 = 90;
        }

        public static final class CoralSetpoints{
                public static final double kFeederStation = 1100;
                public static final double kReefPlaceL123 = -1100;
                public static final double kReefPlaceL4 = -2000;
                public static final double kStop = 0;
        }

        public static final class AlgaeSetpoints{
                public static final double kReefPickUpL123 = -1100;
                public static final double kDiliver = 2000;
                public static final double kStop = 0;
        }


        /**
         * Command to set the subsystem setpoint. This will set the arm and elevator to
         * their predefined
         * positions for the given setpoint.
         */
        public Command setSetpointCommand(Setpoint setpoint) {
                return Commands.runOnce(
                                () -> {
                                        switch (setpoint) {
                                                case kFeederStation:
                                                        m_arm.setGoalDegrees(ArmSetpoints.kFeederStation);
                                                        m_elevator.setGoalInches(ElevatorSetpoints.kFeederStation);
                                                        

                                                        break;
                                                case kLevel1:
                                                        m_arm.setGoalDegrees(ArmSetpoints.kLevel1);
                                                        m_elevator.setGoalInches(ElevatorSetpoints.kLevel1);
                                                       
                                                        break;
                                                case kLevel2:
                                                        m_arm.setGoalDegrees(ArmSetpoints.kLevel2);
                                                        m_elevator.setGoalInches(ElevatorSetpoints.kLevel2);
                                                        
                                                        break;
                                                case kLevel3:
                                                m_arm.setGoalDegrees(ArmSetpoints.kLevel3);
                                                m_elevator.setGoalInches(ElevatorSetpoints.kLevel3);
                                               
                                                        break;
                                                case kLevel4:
                                                m_arm.setGoalDegrees(ArmSetpoints.kLevel4);
                                                m_elevator.setGoalInches(ElevatorSetpoints.kLevel4);
                                                
                                                        break;
                                        }
                                });
        }
}
