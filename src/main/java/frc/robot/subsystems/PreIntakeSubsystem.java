package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDConstants;
import monologue.Annotations.Log;
import monologue.Logged;

public class PreIntakeSubsystem extends SubsystemBase implements Logged {

    public final SparkMax preIntakeMotor = new SparkMax(CANIDConstants.preIntakeMotorID, MotorType.kBrushless);

    private SparkClosedLoopController preintakeClosedLoopController = preIntakeMotor.getClosedLoopController();

    @Log(key = "alert warning")
    private Alert allWarnings = new Alert("AllWarnings", AlertType.kWarning);
    @Log(key = "alert error")
    private Alert allErrors = new Alert("AllErrors", AlertType.kError);
    @Log(key = "alert sticky fault")
    private Alert allStickyFaults = new Alert("AllStickyFaults", AlertType.kError);

    SparkMaxConfig preintakeConfig;

    public boolean atUpperLimit;

    public boolean atLowerLimit;

    public double angleToleranceDeg = 10;

    public boolean presetOnce;

    public double gearReduction = 10.;
    double degperencderrev = (360) / gearReduction;

    double posConvFactor = degperencderrev;

    double velConvFactor = posConvFactor / 60;

    double maxmotorrps = 5700 / 60;

    double maxdegpersec = degperencderrev * maxmotorrps;//

    /*
     * ( (value that goes up) - (value that goes down) )/ 2 = ks .4up .04 down
     * ( (value that goes up) + (value that goes down) )/2 = kg
     */

    public double preintakeKp = 0.03;

    public final double preintakeKi = 0.;
    public final double preintakeKd = 0;

    /**
     * Angles are set so that 90 degrees is with the preintake balanced over
     * center
     * This means kg will act equally on both sides of top center
     * 
     */

    public final double minAngle = 0;
    public final double maxAngle = 45;

    public double targetRadians;
    @Log(key = "preintakeff")
    private double preintakeff;

    private double goalDegrees;

    public PreIntakeSubsystem() {

        SmartDashboard.putNumber("Arm/Values/maxdegpersec", maxdegpersec);
        SmartDashboard.putNumber("Arm/Values/poscf", posConvFactor);

        preintakeConfig = new SparkMaxConfig();

        preintakeConfig
                .inverted(false)
                .idleMode(IdleMode.kBrake);

        preintakeConfig.encoder
                .positionConversionFactor(posConvFactor)
                .velocityConversionFactor(velConvFactor);

        preintakeConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Set PID values for position control
                .p(preintakeKp)
                .outputRange(-1, 1);

        preintakeConfig.limitSwitch.forwardLimitSwitchEnabled(false);

        preintakeConfig.softLimit.forwardSoftLimit(maxAngle)
                .reverseSoftLimit(minAngle)
                .forwardSoftLimitEnabled(false)
                .reverseSoftLimitEnabled(false);

        preintakeConfig.signals.primaryEncoderPositionPeriodMs(5);

        preIntakeMotor.configure(preintakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        preIntakeMotor.getEncoder().setPosition(0);

        goalDegrees = 0;

    }

    public boolean getActiveFault() {
        return preIntakeMotor.hasActiveFault();
    }

    public boolean getStickyFault() {
        return preIntakeMotor.hasStickyFault();
    }

    public boolean getWarnings() {
        return preIntakeMotor.hasActiveWarning();
    }

    public Command positionCommand() {
        return Commands.run(() -> preintakeClosedLoopController.setReference(goalDegrees, ControlType.kPosition));
    }

    public Command jogMotorCommand(DoubleSupplier speed) {
        return Commands
                .run(() -> preIntakeMotor.setVoltage(speed.getAsDouble() * RobotController.getBatteryVoltage()));
    }

    public Command stopMotorCommand() {
        return Commands.runOnce(() -> preIntakeMotor.stopMotor());

    }

    @Override
    public void periodic() {

        // This method will be called once per scheduler run
        allWarnings.set(getWarnings());
        allErrors.set(getActiveFault());
        allStickyFaults.set(getStickyFault());

        atUpperLimit = getAngle() > maxAngle;
        atLowerLimit = getAngle() < minAngle;
        SmartDashboard.putNumber("FIM/pos", preIntakeMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("FIM/vel", preIntakeMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("FIM/volts",
                preIntakeMotor.getAppliedOutput() * RobotController.getBatteryVoltage());

    }

    @Override
    public void simulationPeriodic() {

    }

    public void resetEncoder(double val) {
        preIntakeMotor.getEncoder().setPosition(val);
    }

    public void setTolerance(double toleranceDeg) {
        angleToleranceDeg = toleranceDeg;
    }

    public Command setGoalDegreesCommand(double targetDegrees) {
        return Commands.runOnce(() -> goalDegrees = targetDegrees);

    }

    public double getMotorEncoderAngleRadians() {
        return preIntakeMotor.getEncoder().getPosition();
    }

    @Log.NT(key = "motor degrees")
    public double getMotorDegrees() {
        return Units.radiansToDegrees(preIntakeMotor.getEncoder().getPosition());
    }

    public double getMotorEncoderRadsPerSec() {
        return preIntakeMotor.getEncoder().getVelocity();
    }

    public double getMotorEncoderDegsPerSec() {
        return Units.radiansToDegrees(preIntakeMotor.getEncoder().getVelocity());
    }

    @Log(key = "angle")
    public double getAngle() {
        return preIntakeMotor.getEncoder().getPosition();

    }

    public double getRadsPerSec() {
        return preIntakeMotor.getEncoder().getVelocity();
    }

    @Log.NT(key = "preintake degrees per sec")
    public double getDegreesPerSec() {
        return Units.radiansToDegrees(preIntakeMotor.getEncoder().getVelocity());
    }

    public boolean onPlusSoftwareLimit() {
        return preIntakeMotor.getEncoder().getPosition() >= preIntakeMotor.configAccessor.softLimit
                .getForwardSoftLimit();
    }

    public boolean onMinusSoftwareLimit() {
        return preIntakeMotor.getEncoder().getPosition() <= preIntakeMotor.configAccessor.softLimit
                .getReverseSoftLimit();
    }

    public boolean onPlusHardwareLimit() {
        return preIntakeMotor.getForwardLimitSwitch().isPressed();
    }

    public boolean onMinusHardwareLimit() {
        return preIntakeMotor.getReverseLimitSwitch().isPressed();
    }

    @Log(key = "on limit")
    public boolean onLimit() {
        return onPlusHardwareLimit() || onMinusHardwareLimit() ||
                onPlusSoftwareLimit() || onMinusSoftwareLimit();
    }

    public void stop() {
        preIntakeMotor.setVoltage(0);
    }

    public double getAmps() {
        return preIntakeMotor.getOutputCurrent();
    }

    public boolean isBraked() {
        return preIntakeMotor.configAccessor.getIdleMode() == IdleMode.kBrake;
    }

    public boolean getSoftwareLimitsEnabled() {
        return preIntakeMotor.configAccessor.softLimit.getForwardSoftLimitEnabled()
                || preIntakeMotor.configAccessor.softLimit.getReverseSoftLimitEnabled();
    }

    public boolean getStickyFaults() {
        return preIntakeMotor.hasStickyFault();
    }

    public Command clearStickyFaultsCommand() {
        return Commands.runOnce(() -> preIntakeMotor.clearFaults());
    }

}
