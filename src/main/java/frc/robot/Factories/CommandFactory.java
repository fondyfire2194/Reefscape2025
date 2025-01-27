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
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/** Add your docs here. */
public class CommandFactory {

        SwerveSubsystem m_swerve;

        public CommandFactory(SwerveSubsystem swerve, ElevatorSubsystem elevator, ArmSubsystem arm) {
                m_swerve = swerve;
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
                public static final int kFeederStation = 0;
                public static final int kLevel1 = 0;
                public static final int kLevel2 = 0;
                public static final int kLevel3 = 100;
                public static final int kLevel4 = 150;
        }

        public static final class ArmSetpoints {
                public static final double kFeederStation = 33;
                public static final double kLevel1 = 0;
                public static final double kLevel2 = 2;
                public static final double kLevel3 = 2;
                public static final double kLevel4 = 19;
        }

        /**
         * Command to set the subsystem setpoint. This will set the arm and elevator to
         * their predefined
         * positions for the given setpoint.
         */
        public Command setSetpointCommand(Setpoint setpoint, ElevatorSubsystem elevator, ArmSubsystem arm) {
                return Commands.runOnce(
                                () -> {
                                        switch (setpoint) {
                                                case kFeederStation:
                                                        arm.armCurrentTarget = ArmSetpoints.kFeederStation;
                                                        elevator.elevatorCurrentTarget = ElevatorSetpoints.kFeederStation;
                                                        break;
                                                case kLevel1:
                                                        arm.armCurrentTarget = ArmSetpoints.kLevel1;
                                                        elevator.elevatorCurrentTarget = ElevatorSetpoints.kLevel1;
                                                        break;
                                                case kLevel2:
                                                        arm.armCurrentTarget = ArmSetpoints.kLevel2;
                                                        elevator.elevatorCurrentTarget = ElevatorSetpoints.kLevel2;
                                                        break;
                                                case kLevel3:
                                                        arm.armCurrentTarget = ArmSetpoints.kLevel3;
                                                        elevator.elevatorCurrentTarget = ElevatorSetpoints.kLevel3;
                                                        break;
                                                case kLevel4:
                                                        arm.armCurrentTarget = ArmSetpoints.kLevel4;
                                                        elevator.elevatorCurrentTarget = ElevatorSetpoints.kLevel4;
                                                        break;
                                        }
                                });
        }
}
