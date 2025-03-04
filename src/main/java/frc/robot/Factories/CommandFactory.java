// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Factories;

import java.util.Optional;

import org.dyn4j.geometry.Triangle;

import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GamepieceSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.utils.LedStrip;

/** Add your docs here. */
public class CommandFactory {

        SwerveSubsystem m_swerve;
        ElevatorSubsystem m_elevator;
        ArmSubsystem m_arm;
        GamepieceSubsystem m_gamepieces;
        static CommandXboxController m_dr;
        static CommandXboxController m_codr;

        LedStrip m_ls;

        public CommandFactory(SwerveSubsystem swerve, ElevatorSubsystem elevator, ArmSubsystem arm,
                        GamepieceSubsystem gamepieces, LedStrip ls, CommandXboxController dr,
                        CommandXboxController codr) {
                m_swerve = swerve;
                m_dr = dr;
                m_codr = codr;
                m_arm = arm;
                m_elevator = elevator;
                m_gamepieces = gamepieces;
                m_ls = ls;

              
        }

        public Command deliverAlgaeToBargeCommand(double delaySecs) {
                return Commands.parallel(
                                m_arm.setGoalDegreesCommand(ArmSetpoints.kBargeDeliver),
                                Commands.sequence(
                                                Commands.waitSeconds(delaySecs),
                                                m_gamepieces.deliverAlgaeToBargeCommand()));

        }

        public static Command rumbleDriver(RumbleType type, double timeout) {
                return Commands.sequence(
                                Commands.race(
                                                Commands.either(
                                                                Commands.run(() -> m_dr.getHID().setRumble(type,
                                                                                1.0)),
                                                                Commands.runOnce(() -> SmartDashboard.putString("BUZZ",
                                                                                "BUZZ")),
                                                                () -> RobotBase.isReal()),
                                                Commands.waitSeconds(timeout)),
                                Commands.runOnce(() -> m_dr.getHID().setRumble(RumbleType.kBothRumble, 0.0)));
        }

        public static Command rumbleCoDriver(RumbleType type, double timeout) {
                return Commands.sequence(
                                Commands.race(
                                                Commands.either(
                                                                Commands.run(() -> m_codr.getHID().setRumble(type,
                                                                                1.0)),
                                                                Commands.runOnce(() -> SmartDashboard.putString("BUZZ",
                                                                                "BUZZ")),
                                                                () -> RobotBase.isReal()),
                                                Commands.waitSeconds(timeout)),
                                Commands.runOnce(() -> m_codr.getHID().setRumble(RumbleType.kBothRumble, 0.0)));
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
                kCoralStation,
                kLevel1,
                kLevel2,
                kLevel3,
                kLevel4,
                kProcessorDeliver,
                KAlgaeDeliverBarge,
                kAlgaePickUpL3,
                KAlgaePickUpL2;
        }

        public static final class ElevatorSetpoints {
                public static final int kHome = 0;
                public static final int kProcessorDeliver = 5;
                public static final int kCoralStation = 0;
                public static final int kLevel1 = 15;
                public static final int kLevel2 = 20;
                public static final int kLevel3 = 40;
                public static final int kLevel4 = 60;
                public static final int kBarge = 70;
        }

        public static final class ArmSetpoints {
                public static final int kTravel = 100;
                public static final int kProcessorDeliver = -100;
                public static final int kBargeDeliver = -100;
                public static final double kCoralStation = 132;
                public static final double kLevel1 = 97;
                public static final double kLevel2 = 95;
                public static final double kLevel3 = 94;
                public static final double kLevel4 = 90;
                public static final double kAlgaeIntake = -100;

        }

        public static final class CoralRPMSetpoints {
                public static final double kCoralIntakeMotorRPM = 1100;
                public static final double kCoralStation = 1100;
                public static final double kReefPlaceL123 = 1100;
                public static final double kReefPlaceL4 = 2000;
                public static final double kStop = 0;
        }

        public static final class AlgaeRPMSetpoints {
                public static final double kReefPickUpL123 = -.25;
                public static final double kProcessorDeliver = 2000;
                public static final double kBargeDeliver = 2000;
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
                                                case kCoralStation:
                                                        m_arm.setGoalDegrees(ArmSetpoints.kCoralStation);
                                                        m_elevator.setGoalInchesWithArmCheck(
                                                                        ElevatorSetpoints.kCoralStation);
                                                        break;
                                                case kLevel1:
                                                        m_arm.setGoalDegrees(ArmSetpoints.kLevel1);
                                                        m_elevator.setGoalInchesWithArmCheck(ElevatorSetpoints.kLevel1);
                                                        break;
                                                case kLevel2:
                                                        m_arm.setGoalDegrees(ArmSetpoints.kLevel2);
                                                        m_elevator.setGoalInchesWithArmCheck(ElevatorSetpoints.kLevel2);
                                                        break;
                                                case kLevel3:
                                                        m_arm.setGoalDegrees(ArmSetpoints.kLevel3);
                                                        m_elevator.setGoalInchesWithArmCheck(ElevatorSetpoints.kLevel3);
                                                        break;
                                                case kLevel4:
                                                        m_arm.setGoalDegrees(ArmSetpoints.kLevel4);
                                                        m_elevator.setGoalInchesWithArmCheck(ElevatorSetpoints.kLevel4);
                                                        break;
                                                case kProcessorDeliver:
                                                        m_arm.setGoalDegrees(ArmSetpoints.kProcessorDeliver);
                                                        m_elevator.setGoalInchesWithArmCheck(
                                                                        ElevatorSetpoints.kProcessorDeliver);
                                                        break;
                                                case KAlgaeDeliverBarge:
                                                        m_arm.setGoalDegrees(ArmSetpoints.kBargeDeliver);
                                                        m_elevator.setGoalInchesWithArmCheck(ElevatorSetpoints.kBarge);
                                                        break;
                                                case KAlgaePickUpL2:
                                                        m_arm.setGoalDegrees(ArmSetpoints.kAlgaeIntake);
                                                        m_elevator.setGoalInchesWithArmCheck(ElevatorSetpoints.kLevel2);
                                                        break;
                                                case kAlgaePickUpL3:
                                                        m_arm.setGoalDegrees(ArmSetpoints.kAlgaeIntake);
                                                        m_elevator.setGoalInchesWithArmCheck(ElevatorSetpoints.kLevel3);
                                                        break;
                                        }

                                })

                                .andThen(Commands.runOnce(() -> m_ls.setViewThreeSolidColor(setpoint.ordinal())));
        }
}
