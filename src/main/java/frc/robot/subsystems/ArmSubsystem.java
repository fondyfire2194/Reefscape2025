package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
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

    SparkMaxConfig armConfig;

    public boolean armMotorConnected;

    public double appliedOutput;

    public boolean inIZone;

    public double armVolts;

    public double appliedVolts;

    public boolean atUpperLimit;

    public boolean atLowerLimit;

    public Angle angleToleranceRads = Radians.of(Units.degreesToRadians(1));

    public boolean enableArm;

    public boolean presetOnce;

    private AngularVelocity maxVel = DegreesPerSecond.of(20);
    private AngularAcceleration maxAccel = DegreesPerSecondPerSecond.of(15);

    public final Angle minAngle = Degrees.of(-10.1); // -50.1 deg from horiz
    public final Angle maxAngle = Degrees.of(40.9 + 180); // 40.9 deg from horiz

    private final MutAngle m_angle = Degrees.mutable(0);
    public double kp = 0;
    public double ki = 0;
    public double kd = 0;

    public final double armKg = 0.1;
    public final double armKs = 0.31;
    public final double armKv = 2;// volts per deg per sec so 12/max = 12/5=2.4
    public final double armKa = 0;

    public final double armKp = 30;
    public final double armKi = 0.5;
    public final double armKd = 0.0;

    private AngularVelocity maxrevpermin = RPM.of(5700);

    public double gearReduction = 60;
    public double armLength = Units.inchesToMeters(40);
    public double armMass = 4.3;
    double radperencderrev = 2 * Math.PI / gearReduction;

    double posConvFactor = radperencderrev;

    double velConvFactor = posConvFactor / 60;

    double maxRadPerSec = velConvFactor * maxrevpermin.in(RPM);//

    double ks = armKs;
    double kv = 12 / maxRadPerSec;// 8
    double kg = armKg;
    double ka = armKa;
    double lastkv = kv;
    boolean tuning = false;

    private int inPositionCtr;

    private double armCurrentTarget;

    public ArmSubsystem() {

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
                .p(0.1)
                .outputRange(-1, 1)
                .maxMotion
                // Set MAXMotion parameters for position control
                .maxVelocity(20)
                .maxAcceleration(100)
                .allowedClosedLoopError(0.25);

        armConfig.limitSwitch.forwardLimitSwitchEnabled(false);

        armConfig.signals.primaryEncoderPositionPeriodMs(5);

        armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        armMotor.getEncoder().setPosition(0);
        resetEncoder(0);
        armCurrentTarget = minAngle.in(Radians);

    }

    @Override
    public void periodic() {

        SD.sd2("Arm/ArmPosition", getAngle().in(Degrees));

        atUpperLimit = getAngle().gte(maxAngle);
        atLowerLimit = minAngle.lte(getAngle());

        SmartDashboard.putBoolean("Arm/atUpperLimit", atUpperLimit);
        SmartDashboard.putBoolean("Arm/atLowerLimit", atLowerLimit);

    }

    @Override
    public void simulationPeriodic() {

    }

    public boolean atGoal() {
        return Math.abs(armCurrentTarget - getAngle().in(Radians)) < .01;

    }

    public void resetEncoder(double val) {
        armMotor.getEncoder().setPosition(val);
    }

    public void position() {

        if (inPositionCtr != 3)
            inPositionCtr++;

        armClosedLoopController.setReference(
                armCurrentTarget, ControlType.kMAXMotionPositionControl);

    }

    public void setTolerance(Angle toleranceRads) {
        angleToleranceRads = toleranceRads;
    }

    public void setTarget(Angle angle) {
        if (angle.gt(maxAngle))
            angle = maxAngle;
        if (angle.lt(minAngle))
            angle = minAngle;
        armCurrentTarget = angle.in(Radians);
        inPositionCtr = 0;
        SmartDashboard.putNumber("Arm/Target", armCurrentTarget);
    }

    // provides an always down final move to position by first moving
    // UDA parameter rads above goal if target is above actual
    public Command setTargetCommand(double degrees) {
        return Commands.sequence(
                Commands.runOnce(() -> setTolerance(angleToleranceRads)),
                Commands.runOnce(() -> setTarget(Radians.of(degrees))));

    }

    public void incrementArmAngle(Angle incAngle) {
        double temp = armCurrentTarget;
        temp += incAngle.in(Degrees);
        if (temp > (maxAngle.in(Degrees)))
            temp = maxAngle.in(Degrees);
        Angle a = Degrees.of(temp);
        setTarget(a);
    }

    public void decrementArmAngle(Angle decAngle) {
        double temp = armCurrentTarget;
        temp -= decAngle.in(Degrees);
        if (temp < (minAngle.in(Degrees)))
            temp = minAngle.in(Degrees);
        Angle a = Degrees.of(temp);
        setTarget(a);
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
        double offsetAngle = rawAngle;
        m_angle.mut_replace(offsetAngle, Radians); // NOTE: the encoder must be configured with distancePerPulse in //
                                                   // // terms // // of radians
        return m_angle;
    }

    public Angle getAngleError() {
        double diff = armCurrentTarget - getAngle().in(Radians);
        return Radians.of(diff);
    }

    public boolean getAtSetpoint() {
        return Math.abs(getAngleError().in(Radians)) <= (angleToleranceRads.in(Radians));
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

    public Command testCan() {
        return Commands.runOnce(() -> armMotorConnected = false);
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
