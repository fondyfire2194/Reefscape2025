package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.CANIDConstants;
import frc.robot.utils.SD;

public class ArmSubsystem extends SubsystemBase {

    public final SparkMax armMotor = new SparkMax(CANIDConstants.armMotorID, MotorType.kBrushless);

    private SparkClosedLoopController armClosedLoopController = armMotor.getClosedLoopController();

    public ArmFeedforward armfeedforward;

    SparkMaxConfig armConfig;

    public boolean atUpperLimit;

    public boolean atLowerLimit;

    public Angle angleToleranceRads = Radians.of(Units.degreesToRadians(1));

    public boolean presetOnce;

    public final Angle minAngle = Degrees.of(0); // pointing down
    public final Angle maxAngle = Degrees.of(150); // 40.9 deg from horiz

    public double gearReduction = 40;
    public double armLength = Units.inchesToMeters(20);
    public double armMass = Units.lbsToKilograms(4.3);
    double radperencderrev = 2 * Math.PI / gearReduction;

    double posConvFactor = radperencderrev;

    double velConvFactor = posConvFactor / 60;

    double maxmotorrps = 4800 / 60;

    double maxradpersec = radperencderrev * maxmotorrps;

    double maxdegrespersec = Units.radiansToDegrees(maxradpersec);

    public final double armKg = 0.1;
    public final double armKs = 0.1;
    public final double armKv = 10 / maxradpersec;
    public final double armKa = 0;

    public final double armKp = 0.9;
    public final double armKi = 0.;
    public final double armKd = 0;

    public final Angle armStartupOffset =Radians.of(0);

    double TRAJECTORY_VEL = 2;
    double TRAJECTORY_ACCEL = 3;

    private final TrapezoidProfile m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
            TRAJECTORY_VEL, TRAJECTORY_ACCEL));
    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State currentSetpoint = new TrapezoidProfile.State();
    private TrapezoidProfile.State nextSetpoint = new TrapezoidProfile.State();

    private int inPositionCtr;

    public double targetRadians;

    public ArmSubsystem() {

        SmartDashboard.putNumber("Arm/maxdegpersec", maxdegrespersec);
        SmartDashboard.putNumber("Arm/poscf", posConvFactor);
        SmartDashboard.putNumber("Arm/maxradpersec", maxradpersec);
     

        armConfig = new SparkMaxConfig();

        armConfig
                .inverted(true)
                .idleMode(IdleMode.kBrake);

        armConfig.encoder
                .positionConversionFactor(posConvFactor)
                .velocityConversionFactor(velConvFactor);

        armConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Set PID values for position control
                .p(armKp)
                .outputRange(-1, 1).maxMotion
                // Set MAXMotion parameters for position control
                .maxVelocity(10)
                .maxAcceleration(10)
                .allowedClosedLoopError(0.25);

        armConfig.limitSwitch.forwardLimitSwitchEnabled(false);

        armConfig
        .softLimit.forwardSoftLimit(2.5)
        .reverseSoftLimit(0)
        .forwardSoftLimitEnabled(false)
        .reverseSoftLimitEnabled(false);


        armConfig.signals.primaryEncoderPositionPeriodMs(5);

        armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        armfeedforward = new ArmFeedforward(armKa, armKg, armKv);

        armMotor.getEncoder().setPosition(0);
        resetEncoder(Units.degreesToRadians(10));
        setGoalRadians(0);

    }

    @Override
    public void periodic() {

        SD.sd2("Arm/ArmPosition", getAngle().in(Degrees));

        atUpperLimit = getAngle().gte(maxAngle);
        atLowerLimit = getAngle().lte(minAngle);

        SmartDashboard.putBoolean("Arm/atUpperLimit", atUpperLimit);
        SmartDashboard.putBoolean("Arm/atLowerLimit", atLowerLimit);

    }

    @Override
    public void simulationPeriodic() {

    }

    // public boolean atGoal() {
    // return Math.abs( - getAngle().in(Radians)) < .01;
    // }

    public void resetEncoder(double val) {
        armMotor.getEncoder().setPosition(val);
    }

    public void position() {
        if (inPositionCtr != 3)
            inPositionCtr++;
        // Send setpoint to spark max controller
        nextSetpoint = m_profile.calculate(.02, currentSetpoint, m_goal);

        SmartDashboard.putNumber("Arm/setpos", nextSetpoint.position);
        SmartDashboard.putNumber("Arm/setvel", nextSetpoint.velocity);

        double leftff = armfeedforward.calculate(getAngle().in(Radians), nextSetpoint.velocity);

        currentSetpoint = nextSetpoint;

        SmartDashboard.putNumber("Elevator/ff", leftff);

        armClosedLoopController.setReference(
                nextSetpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, leftff, ArbFFUnits.kVoltage);

    }

    public void setTolerance(Angle toleranceRads) {
        angleToleranceRads = toleranceRads;
    }

    public void setGoalRadians(double targetRads) {
        m_goal.position = targetRads;
        currentSetpoint.position = getAngle().in(Radians);
        targetRadians = targetRads;
        SmartDashboard.putNumber("Arm/targetdeg", Units.radiansToDegrees(targetRads));
        // inPositionCtr = 0;
    }

    public void setGoalDegrees(double targetDegrees) {
        double targetRads = Units.degreesToRadians(targetDegrees);
        setGoalRadians(targetRads);

    }

    public Command setGoalDegreesCommand(double targetDegrees) {
        return Commands.runOnce(() -> setGoalDegrees(targetDegrees));

    }

    public double getMotorEncoderAngleRadians() {
        return armMotor.getEncoder().getPosition();
    }

    public double getMotorDegrees() {
        return Units.radiansToDegrees(armMotor.getEncoder().getPosition());
    }

    public double getMotorEncoderRadsPerSec() {
        return armMotor.getEncoder().getVelocity();
    }

    public double getMotorEncoderDegsPerSec() {
        return Units.radiansToDegrees(armMotor.getEncoder().getVelocity());
    }

    public Angle getAngle() {
        double rawAngle = armMotor.getEncoder().getPosition();
        return Radians.of(rawAngle);
    }

    public double getRadsPerSec() {
        return armMotor.getEncoder().getVelocity();
    }

    public double getDegreesPerSec() {
        return Units.radiansToDegrees(armMotor.getEncoder().getVelocity());
    }

    public boolean onPlusSoftwareLimit() {
        return armMotor.getEncoder().getPosition() >= armMotor.configAccessor.softLimit.getForwardSoftLimit();
    }

    public boolean onMinusSoftwareLimit() {
        return armMotor.getEncoder().getPosition() <= armMotor.configAccessor.softLimit.getReverseSoftLimit();
    }

    public boolean onPlusHardwareLimit() {
        return armMotor.getForwardLimitSwitch().isPressed();
    }

    public boolean onMinusHardwareLimit() {
        return armMotor.getReverseLimitSwitch().isPressed();
    }

    public boolean onLimit() {
        return onPlusHardwareLimit() || onMinusHardwareLimit() ||
                onPlusSoftwareLimit() || onMinusSoftwareLimit();
    }

    public void stop() {
        armMotor.setVoltage(0);
    }

    public double getAmps() {
        return armMotor.getOutputCurrent();
    }

    public boolean isBraked() {
        return armMotor.configAccessor.getIdleMode() == IdleMode.kBrake;
    }

    public boolean getSoftwareLimitsEnabled() {
        return armMotor.configAccessor.softLimit.getForwardSoftLimitEnabled()
                || armMotor.configAccessor.softLimit.getReverseSoftLimitEnabled();
    }

    public boolean getStickyFaults() {
        return armMotor.hasStickyFault();
    }

    public Command clearStickyFaultsCommand() {
        return Commands.runOnce(() -> armMotor.clearFaults());
    }

    private SysIdRoutine sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                    (volts) -> {
                        armMotor.setVoltage(volts.in(Volts));
                    },
                    null,
                    this));

    public Command quasistaticForward() {
        return sysIdRoutine.quasistatic(Direction.kForward);
    }

    public Command quasistaticBackward() {
        return sysIdRoutine.quasistatic(Direction.kReverse);
    }

    public Command dynamicForward() {
        return sysIdRoutine.dynamic(Direction.kForward);
    }

    public Command dynamicBackward() {
        return sysIdRoutine.dynamic(Direction.kReverse);
    }

}
