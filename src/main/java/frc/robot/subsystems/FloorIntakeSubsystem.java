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

public class FloorIntakeSubsystem extends SubsystemBase implements Logged {

    public final SparkMax floorIntakeMotor = new SparkMax(CANIDConstants.floorIntakeMotorID, MotorType.kBrushless);

    private SparkClosedLoopController floorintakeClosedLoopController = floorIntakeMotor.getClosedLoopController();

    @Log(key = "alert warning")
    private Alert allWarnings = new Alert("AllWarnings", AlertType.kWarning);
    @Log(key = "alert error")
    private Alert allErrors = new Alert("AllErrors", AlertType.kError);
    @Log(key = "alert sticky fault")
    private Alert allStickyFaults = new Alert("AllStickyFaults", AlertType.kError);

    SparkMaxConfig floorintakeConfig;

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

    public double floorintakeKp = 0.03;

    public final double floorintakeKi = 0.;
    public final double floorintakeKd = 0;

    /**
     * Angles are set so that 90 degrees is with the floorintake balanced over
     * center
     * This means kg will act equally on both sides of top center
     * 
     */

    public final double minAngle = 0;
    public final double maxAngle = 45;

    private int inPositionCtr;

    public double targetRadians;
    @Log(key = "floorintakeff")
    private double floorintakeff;

    private double goalDegrees;

    public FloorIntakeSubsystem() {

        SmartDashboard.putNumber("Arm/Values/maxdegpersec", maxdegpersec);
        SmartDashboard.putNumber("Arm/Values/poscf", posConvFactor);

        floorintakeConfig = new SparkMaxConfig();

        floorintakeConfig
                .inverted(false)
                .idleMode(IdleMode.kBrake);

        floorintakeConfig.encoder
                .positionConversionFactor(posConvFactor)
                .velocityConversionFactor(velConvFactor);

        floorintakeConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Set PID values for position control
                .p(floorintakeKp)
                .outputRange(-1, 1);

        floorintakeConfig.limitSwitch.forwardLimitSwitchEnabled(false);

        floorintakeConfig.softLimit.forwardSoftLimit(maxAngle)
                .reverseSoftLimit(minAngle)
                .forwardSoftLimitEnabled(false)
                .reverseSoftLimitEnabled(false);

        floorintakeConfig.signals.primaryEncoderPositionPeriodMs(5);

        floorIntakeMotor.configure(floorintakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        floorIntakeMotor.getEncoder().setPosition(0);

        goalDegrees = 0;

    }

    public boolean getActiveFault() {
        return floorIntakeMotor.hasActiveFault();
    }

    public boolean getStickyFault() {
        return floorIntakeMotor.hasStickyFault();
    }

    public boolean getWarnings() {
        return floorIntakeMotor.hasActiveWarning();
    }

    public Command positionCommand() {
        return Commands.run(() -> floorintakeClosedLoopController.setReference(goalDegrees, ControlType.kPosition));
    }

    public Command jogMotorCommand(DoubleSupplier speed) {
        return Commands
                .run(() -> floorIntakeMotor.setVoltage(speed.getAsDouble() * RobotController.getBatteryVoltage()));
    }

    public Command stopMotorCommand() {
        return Commands.runOnce(() -> floorIntakeMotor.stopMotor());

    }

    @Override
    public void periodic() {

        // This method will be called once per scheduler run
        allWarnings.set(getWarnings());
        allErrors.set(getActiveFault());
        allStickyFaults.set(getStickyFault());

        atUpperLimit = getAngle() > maxAngle;
        atLowerLimit = getAngle() < minAngle;
        SmartDashboard.putNumber("FIM/pos", floorIntakeMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("FIM/vel", floorIntakeMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("FIM/volts",
                floorIntakeMotor.getAppliedOutput() * RobotController.getBatteryVoltage());

    }

    @Override
    public void simulationPeriodic() {

    }

    public void resetEncoder(double val) {
        floorIntakeMotor.getEncoder().setPosition(val);
    }

    public void setTolerance(double toleranceDeg) {
        angleToleranceDeg = toleranceDeg;
    }

    public Command setGoalDegreesCommand(double targetDegrees) {
        return Commands.runOnce(() -> goalDegrees = targetDegrees);

    }

    public double getMotorEncoderAngleRadians() {
        return floorIntakeMotor.getEncoder().getPosition();
    }

    @Log.NT(key = "motor degrees")
    public double getMotorDegrees() {
        return Units.radiansToDegrees(floorIntakeMotor.getEncoder().getPosition());
    }

    public double getMotorEncoderRadsPerSec() {
        return floorIntakeMotor.getEncoder().getVelocity();
    }

    public double getMotorEncoderDegsPerSec() {
        return Units.radiansToDegrees(floorIntakeMotor.getEncoder().getVelocity());
    }

    @Log(key = "angle")
    public double getAngle() {
        return floorIntakeMotor.getEncoder().getPosition();

    }

    public double getRadsPerSec() {
        return floorIntakeMotor.getEncoder().getVelocity();
    }

    @Log.NT(key = "floorintake degrees per sec")
    public double getDegreesPerSec() {
        return Units.radiansToDegrees(floorIntakeMotor.getEncoder().getVelocity());
    }

    public boolean onPlusSoftwareLimit() {
        return floorIntakeMotor.getEncoder().getPosition() >= floorIntakeMotor.configAccessor.softLimit
                .getForwardSoftLimit();
    }

    public boolean onMinusSoftwareLimit() {
        return floorIntakeMotor.getEncoder().getPosition() <= floorIntakeMotor.configAccessor.softLimit
                .getReverseSoftLimit();
    }

    public boolean onPlusHardwareLimit() {
        return floorIntakeMotor.getForwardLimitSwitch().isPressed();
    }

    public boolean onMinusHardwareLimit() {
        return floorIntakeMotor.getReverseLimitSwitch().isPressed();
    }

    @Log(key = "on limit")
    public boolean onLimit() {
        return onPlusHardwareLimit() || onMinusHardwareLimit() ||
                onPlusSoftwareLimit() || onMinusSoftwareLimit();
    }

    public void stop() {
        floorIntakeMotor.setVoltage(0);
    }

    public double getAmps() {
        return floorIntakeMotor.getOutputCurrent();
    }

    public boolean isBraked() {
        return floorIntakeMotor.configAccessor.getIdleMode() == IdleMode.kBrake;
    }

    public boolean getSoftwareLimitsEnabled() {
        return floorIntakeMotor.configAccessor.softLimit.getForwardSoftLimitEnabled()
                || floorIntakeMotor.configAccessor.softLimit.getReverseSoftLimitEnabled();
    }

    public boolean getStickyFaults() {
        return floorIntakeMotor.hasStickyFault();
    }

    public Command clearStickyFaultsCommand() {
        return Commands.runOnce(() -> floorIntakeMotor.clearFaults());
    }

}
