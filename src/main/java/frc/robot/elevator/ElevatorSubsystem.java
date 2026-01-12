// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elevator;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PhysicalConstants.ElevatorConstants;
import frc.robot.constants.PhysicalConstants.ElevatorConstants.Slot0Gains;
import frc.robot.constants.PhysicalConstants.RobotConstants;
import frc.robot.constants.VirtualConstants.ElevatorPositions;
import org.littletonrobotics.junction.Logger;

/** Subsystem to control Elevator motion */
public class ElevatorSubsystem extends SubsystemBase {
    // Use Bill Pugh Singleton Pattern for efficient lazy initialization (thread-safe !)
    private static class ElevatorSubsystemHolder {
        private static final ElevatorSubsystem INSTANCE = new ElevatorSubsystem();
    }

    /** Always use this method to get the singleton instance of this subsystem. */
    public static ElevatorSubsystem getInstance() {
        return ElevatorSubsystemHolder.INSTANCE;
    }

    private TalonFX leftMotor = new TalonFX(ElevatorConstants.LEFT_MOTOR_ID, RobotConstants.CAN_BUS);
    private TalonFX rightMotor = new TalonFX(ElevatorConstants.RIGHT_MOTOR_ID, RobotConstants.CAN_BUS);

    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);
    private final VoltageOut voltageOut = new VoltageOut(0);
    private final Follower FOLLOW_RIGHT = new Follower(ElevatorConstants.RIGHT_MOTOR_ID, true);
    private final Follower FOLLOW_LEFT = new Follower(ElevatorConstants.LEFT_MOTOR_ID, true);

    private double lastPosition = Double.NaN;
    private double lastRotorVelocity = Double.NaN;

    private double lastSetGoal;

    private ElevatorSubsystem() {
        super("ElevatorSubsystem");

        this.configureMotors();

        this.rightMotor.getPosition().setUpdateFrequency(50);
        this.rightMotor.getVelocity().setUpdateFrequency(50);

        this.leftMotor.setControl(this.FOLLOW_RIGHT);

        this.lastSetGoal = getPosition();
    }

    @Override
    public void periodic() {
        double position = getPosition();
        double rotorVelocity = getRotorVelocity();

        if (position != this.lastPosition) {
            SmartDashboard.putNumber("Elevator/Position", position);
            Logger.recordOutput("Elevator/Position", position);
            this.lastPosition = position;
        }

        if (rotorVelocity != this.lastRotorVelocity) {
            Logger.recordOutput("Elevator/RotorVelocity", getRotorVelocity());
            this.lastRotorVelocity = rotorVelocity;
        }

        boolean inputToggled = SmartDashboard.getBoolean("Elevator/ToggleInputSlider", false);

        if (!inputToggled) {
            SmartDashboard.putNumber("Elevator/InputSlider", position);
        }

        if (DriverStation.isEnabled()) {
            ControlRequest appliedControl = this.rightMotor.getAppliedControl();
            String controlName = appliedControl.getControlInfo().get("Name");
            boolean leftMotor = false;

            // This is supposedly faster than an if statement since it's a string
            switch (controlName) {
                case "Follower":
                    appliedControl = this.leftMotor.getAppliedControl();
                    controlName = appliedControl.getControlInfo().get("Name");
                    leftMotor = true;
                    break;
            }

            switch (controlName) {
                case "VoltageOut":
                    if (leftMotor) {
                        this.leftMotor.setControl(((VoltageOut) appliedControl)
                            // .withLimitForwardMotion(atUpperLimit)
                            // .withLimitReverseMotion(atLowerLimit())
                        );
                        this.rightMotor.setControl(this.FOLLOW_LEFT);
                    }
                    else {
                        this.rightMotor.setControl(((VoltageOut) appliedControl)
                            // .withLimitForwardMotion(atUpperLimit)
                            // .withLimitReverseMotion(atLowerLimit())
                        );
                        this.leftMotor.setControl(this.FOLLOW_RIGHT);
                    }
                    break;

                case "MotionMagicVoltage":
                    if (leftMotor) {
                        this.leftMotor.setControl(((MotionMagicVoltage) appliedControl)
                            // .withLimitForwardMotion(atUpperLimit)
                            // .withLimitReverseMotion(atLowerLimit())
                        );
                        this.rightMotor.setControl(this.FOLLOW_LEFT);
                    }
                    else {
                        this.rightMotor.setControl(((MotionMagicVoltage) appliedControl)
                            // .withLimitForwardMotion(atUpperLimit)
                            // .withLimitReverseMotion(atLowerLimit())
                        );
                        this.leftMotor.setControl(this.FOLLOW_RIGHT);
                    }
                    break;
            }

            Command currentCommand = getCurrentCommand();

            /* if (atUpperLimit) {
                if (currentCommand != null) {
                    CommandScheduler.getInstance().cancel(currentCommand);
                }
                motionMagicPosition(position - 0.01, false, true);
            }
            else */ if (currentCommand != null) {
                SmartDashboard.putBoolean("Elevator/ToggleInputSlider", false);
            }
            else if (inputToggled && currentCommand == null) {
                motionMagicPosition(SmartDashboard.getNumber("Elevator/InputSlider", 0), true, false);
            }
        }
        else {
            SmartDashboard.putBoolean("Elevator/ToggleInputSlider", false);
            setVoltage(0);
        }
    }

    /**
     * A helper method that configures MotionMagic on both motors.
     */
    private void configureMotors() {
        TalonFXConfiguration configuration = new TalonFXConfiguration();

        FeedbackConfigs feedbackConfigs = configuration.Feedback;
        feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        // Sets the gear ratio from the rotor to the mechanism.
        // This gear ratio needs to be exact.
        feedbackConfigs.SensorToMechanismRatio = ElevatorConstants.ROTOR_TO_MECHANISM_RATIO;

        MotorOutputConfigs motorOutputConfigs = configuration.MotorOutput;
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

        // Set Motion Magic gains in slot 0.
        Slot0Configs slot0Configs = configuration.Slot0;
        slot0Configs.GravityType = GravityTypeValue.Elevator_Static;
        slot0Configs.kG = Slot0Gains.kG;
        slot0Configs.kS = Slot0Gains.kS;
        slot0Configs.kV = Slot0Gains.kV;
        slot0Configs.kA = Slot0Gains.kA;
        slot0Configs.kP = Slot0Gains.kP;
        slot0Configs.kI = Slot0Gains.kI;
        slot0Configs.kD = Slot0Gains.kD;

        // Set acceleration and cruise velocity.
        MotionMagicConfigs motionMagicConfigs = configuration.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ElevatorConstants.CRUISE_SPEED;
        motionMagicConfigs.MotionMagicAcceleration = ElevatorConstants.ACCELERATION;

        CurrentLimitsConfigs currentLimitsConfigs = configuration.CurrentLimits;
        currentLimitsConfigs.StatorCurrentLimitEnable = true;
        currentLimitsConfigs.StatorCurrentLimit = ElevatorConstants.STATOR_CURRENT_LIMIT;
        currentLimitsConfigs.SupplyCurrentLimitEnable = true;
        currentLimitsConfigs.SupplyCurrentLimit = 15;
        currentLimitsConfigs.SupplyCurrentLowerTime = 0.5;
        currentLimitsConfigs.SupplyCurrentLowerLimit = 7.5;

        // Motor-specific configurations.
        motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive; // Right motor not inverted.
        this.rightMotor.getConfigurator().apply(configuration);

        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive; // Left motor inverted.
        motionMagicConfigs.MotionMagicCruiseVelocity = ElevatorConstants.SLOW_CRUISE_SPEED;
        motionMagicConfigs.MotionMagicAcceleration = ElevatorConstants.SLOW_ACCELERATION;
        this.leftMotor.getConfigurator().apply(configuration);
    }

    /**
     * Sets the position of the elevator in meters.
     * @param position The position in meters.
     */
    public void setPosition(double position) {
        double convertedPosition = metersToRotation(position);
        this.rightMotor.setPosition(convertedPosition);
        this.leftMotor.setPosition(convertedPosition);
    }

    /**
     * Gets the position of the elevator in meters.
     * @return The position in meters.
     * @apiNote Sources position data from the right motor.
     */
    public double getPosition() {
        return this.rotationsToMeters(this.rightMotor.getPosition().getValueAsDouble());
    }

    /**
     * Returns the velocity of the mechanism in rot/s.
     * @return The velocity of the mechanism.
     */
    public double getRotorVelocity() {
        return this.rightMotor.getVelocity().getValueAsDouble();
    }

    /**
     * Moves the elevator to a position in meters using Motion Magic.
     * @param position The position in meters.
     * @param clamp Whether to clamp the position to the soft stops.
     * @param slow Whether to limit the elevator's max speed and acceleration.
     */
    public void motionMagicPosition(double position, boolean clamp, boolean slow) {
        if (clamp) {
            position = MathUtil.clamp(position, ElevatorPositions.IDLE_HEIGHT, ElevatorConstants.MAX_HEIGHT);
        }

        this.lastSetGoal = position;

        MotionMagicVoltage control = motionMagicVoltage
            .withSlot(0)
            .withPosition(this.metersToRotation(position));

        if (slow) {
            this.leftMotor.setControl(control);
            this.rightMotor.setControl(this.FOLLOW_LEFT);
        }
        else {
            this.rightMotor.setControl(control);
            this.leftMotor.setControl(this.FOLLOW_RIGHT);
        }
    }

    /**
     * Sends a voltage to the motor
     * @param voltage Voltage in between 12 and -12
     */
    public void setVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, -12, 12);

        VoltageOut control = this.voltageOut
            .withOutput(voltage);

        this.rightMotor.setControl(control);
    }

    /**
     * Checks if the current elevator position is within a tolerance of a position.
     * @param position The position to check against.
     * @return Whether the current position is within the tolerance.
     */
    public boolean withinTolerance(double position) {
        return Math.abs(getPosition() - position) <= ElevatorConstants.HEIGHT_TOLERANCE;
    }

    /**
     * Gets the latest set goal.
     * @return The latest set goal.
     */
    public double getLastSetGoal() {
        return this.lastSetGoal;
    }

    /**
     * Gets the current stator current
     * @return The current in amps
     */
    public double getStatorCurrent() {
        return rightMotor.getStatorCurrent().getValueAsDouble();
    }

    /**
     * Converts motor rotations to elevator meters.
     * @param rotations The rotations to convert.
     * @return The meters.
     * @apiNote This method is private and not used by other classes.
     */
    private double rotationsToMeters(double rotations) {
        return Math.PI * ElevatorConstants.ROLLER_DIAMETER * rotations * ElevatorConstants.LINEAR_CONSTANT_MULT;
    }

    /**
     * Converts elevator meters to motor rotations.
     * @param meters The meters to convert.
     * @return The rotations
     * @apiNote This method is private and not used by other classes.
     */
    private double metersToRotation(double meters) {
        return meters / ElevatorConstants.ROLLER_DIAMETER / Math.PI / ElevatorConstants.LINEAR_CONSTANT_MULT;
    }
}