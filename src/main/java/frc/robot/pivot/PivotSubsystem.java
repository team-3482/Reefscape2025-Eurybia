// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.pivot;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PhysicalConstants.PivotConstants;
import frc.robot.constants.PhysicalConstants.PivotConstants.Slot0Gains;
import frc.robot.constants.VirtualConstants.PivotPositionNames;
import org.littletonrobotics.junction.Logger;

import static frc.robot.constants.PhysicalConstants.RobotConstants.CAN_BUS;

/** Subsystem to manage the motion of the Pivot */
public class PivotSubsystem extends SubsystemBase {
    // Use Bill Pugh Singleton Pattern for efficient lazy initialization (thread-safe !)
    private static class PivotSubsystemHolder {
        private static final PivotSubsystem INSTANCE = new PivotSubsystem();
    }

    /** Always use this method to get the singleton instance of this subsystem. */
    public static PivotSubsystem getInstance() {
        return PivotSubsystemHolder.INSTANCE;
    }

    private TalonFX motor = new TalonFX(PivotConstants.MOTOR_ID, CAN_BUS);
    private DigitalInput limitSwitch = new DigitalInput(PivotConstants.LIMIT_SWITCH_ID);
    private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

    private double lastPosition = Double.NaN;
    private boolean lastAtEndstop = true;

    private PivotPositionNames positionName;

    private PivotSubsystem() {
        super("PivotSubsystem");

        configureMotor();
        setPositionHardstop();

        this.motor.getPosition().setUpdateFrequency(50);
        this.motor.getVelocity().setUpdateFrequency(50);

        this.positionName = PivotPositionNames.INTAKE;
    }

    @Override
    public void periodic() {
        double position = getPosition();
        boolean atEndstop = isAtEndstop();

        if (atEndstop != this.lastAtEndstop) {
            SmartDashboard.putBoolean("Pivot/AtEndstop", atEndstop);
            Logger.recordOutput("Pivot/AtEndstop", atEndstop);
            this.lastAtEndstop = atEndstop;
        }

        if (position != this.lastPosition) {
            SmartDashboard.putNumber("Pivot/Position", position);
            Logger.recordOutput("Pivot/Position", position);
            this.lastPosition = position;

            Logger.recordOutput("Pivot/PositionName", positionName);
        }

        boolean inputToggled = SmartDashboard.getBoolean("Pivot/ToggleInputSlider", false);

        if (!inputToggled) {
            SmartDashboard.putNumber("Pivot/InputSlider", position);
        }

        if (DriverStation.isEnabled()) {
            Command currentCommand = getCurrentCommand();
            if (currentCommand != null) {
                SmartDashboard.putBoolean("Pivot/ToggleInputSlider", false);
            } else if (inputToggled) {
                motionMagicPosition(SmartDashboard.getNumber("Pivot/InputSlider", 0), true);
            }
        } else {
            SmartDashboard.putBoolean("Pivot/ToggleInputSlider", false);
            setPivotSpeed(0);
        }
    }

    private void configureMotor() {
        TalonFXConfiguration configuration = new TalonFXConfiguration();

        FeedbackConfigs feedbackConfigs = configuration.Feedback;
        feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        // Sets the gear ration from the rotor to the mechanism.
        // This gear ratio needs to be exact.
        feedbackConfigs.SensorToMechanismRatio = PivotConstants.ROTOR_TO_MECHANISM_RATIO;

        MotorOutputConfigs motorOutputConfigs = configuration.MotorOutput;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

        // Set Motion Magic gains in slot 0.
        Slot0Configs slot0Configs = configuration.Slot0;
        slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;
        slot0Configs.kG = Slot0Gains.kG;
        slot0Configs.kS = Slot0Gains.kS;
        slot0Configs.kV = Slot0Gains.kV;
        slot0Configs.kA = Slot0Gains.kA;
        slot0Configs.kP = Slot0Gains.kP;
        slot0Configs.kI = Slot0Gains.kI;
        slot0Configs.kD = Slot0Gains.kD;

        // Set acceleration and cruise velocity.
        MotionMagicConfigs motionMagicConfigs = configuration.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = PivotConstants.CRUISE_SPEED;
        motionMagicConfigs.MotionMagicAcceleration = PivotConstants.ACCELERATION;

        this.motor.getConfigurator().apply(configuration);
    }

    /**
     * Goes to a position using Motion Magic slot 0.
     * @param position The position for the pivot in degrees.
     * @param clamp Whether to clamp with the soft limits.
     * @apiNote The soft limits in {@link PivotConstants}.
     */
    public void motionMagicPosition(double position, boolean clamp) {
        if (clamp) {
            position = MathUtil.clamp(position, PivotConstants.LOWER_ANGLE_LIMIT, PivotConstants.UPPER_ANGLE_LIMIT);
        }

        MotionMagicVoltage control = motionMagicVoltage
            .withSlot(0)
            .withPosition(Units.degreesToRotations(position));

        this.motor.setControl(control);
    }
    /**
     * Goes to a position using Motion Magic slot 0.
     * @param position The position for the pivot in degrees.
     * @apiNote The position is clamped by the soft limits in {@link PivotConstants}.
     */
    public void motionMagicPosition(double position) {
        motionMagicPosition(position, true);
    }

    /**
     * Sets the speed for the pivot motor.
     * @param speed Speed from -1.0 to 1.0 for the motor.
     * @param safe Whether to respect soft limits. See {@link PivotConstants} for limits.
     */
    public void setPivotSpeed(double speed, boolean safe) {
        if (safe && speed != 0) {
            double position = getPosition();
            if ((speed > 0 && position >= PivotConstants.UPPER_ANGLE_LIMIT)
                || (speed < 0 && position <= PivotConstants.LOWER_ANGLE_LIMIT)) {
                speed = 0;
            }
        }

        this.motor.set(speed);
    }

    /**
     * Sets the speed for the pivot motor.
     * @param speed Speed from -1.0 to 1.0 for the motor.
     * @apiNote Will respect soft limits marked in {@link PivotConstants}.
     */
    public void setPivotSpeed(double speed) {
        setPivotSpeed(speed, true);
    }

    /**
     * Sets the mechanism position of the motor.
     * @param position The position in degrees.
     */
    public void setPosition(double position) {
        position = Units.degreesToRotations(position);
        this.motor.setPosition(position);
    }

    /**
     * Sets the mechanism position of the motor to the lower hard stop.
     * @apiNote See hard stop at {@link PivotConstants#LOWER_ANGLE_LIMIT}
     */
    public void setPositionHardstop() {
        setPosition(0);
    }

    /**
     * Gets the mechanism position of the motor.
     * @return The angle in degrees.
     */
    public double getPosition() {
        return Units.rotationsToDegrees(this.motor.getPosition().getValueAsDouble());
    }

    /**
     * Checks if the current position is within
     * {@link PivotConstants#POSITION_TOLERANCE} of an input position.
     * @param position The position to compare to in rot.
     */
    public boolean withinTolerance(double position) {
        return Math.abs(getPosition() - position) <= PivotConstants.POSITION_TOLERANCE;
    }

    /**
     * Gets the mechanism velocity of the motor.
     * @return The velocity in degrees/s.
     */
    public double getRotorVelocity() {
        return Units.rotationsToDegrees(this.motor.getVelocity().getValueAsDouble());
    }

    /**
     * Finds if moving the elevator is safe and won't crash from pivot in the way
     * @return The boolean where true is safe
     */
    public boolean isSafeToElevate() {
        double position = getPosition();
        return (position >= PivotConstants.MINIMUM_SAFE && position <= PivotConstants.MAXIMUM_SAFE);
    }

    /** Set the position name of the current pivot position
     * @param name the name of the position
     */
    public void setPositionName(PivotPositionNames name) {
        positionName = name;
    }

    /** Get the pivot position name */
    public PivotPositionNames getPositionName() {
        return positionName;
    }

    /** Get the value of the endstop limit switch
     * @return True if the endstop is being pressed
     */
    public boolean isAtEndstop() {
        return !limitSwitch.get();
    }
}