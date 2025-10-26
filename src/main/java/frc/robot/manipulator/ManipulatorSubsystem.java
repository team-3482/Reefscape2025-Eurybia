// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.manipulator;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PhysicalConstants.ManipulatorConstants;
import frc.robot.constants.VirtualConstants.SubsystemStates;
import org.littletonrobotics.junction.Logger;

import static frc.robot.constants.PhysicalConstants.RobotConstants.CAN_BUS;

/** Subsystem to manage the manipulator. */
public class ManipulatorSubsystem extends SubsystemBase {
    // Use Bill Pugh Singleton Pattern for efficient lazy initialization (thread-safe !)
    private static class ManipulatorSubsystemHolder {
        private static final ManipulatorSubsystem INSTANCE = new ManipulatorSubsystem();
    }

    /** Always use this method to get the singleton instance of this subsystem. */
    public static ManipulatorSubsystem getInstance() {
        return ManipulatorSubsystemHolder.INSTANCE;
    }

    private TalonFX coralMotor = new TalonFX(ManipulatorConstants.CORAL_MOTOR_ID, CAN_BUS);
    private TalonFX algaeMotor = new TalonFX(ManipulatorConstants.ALGAE_MOTOR_ID, CAN_BUS);

    private SubsystemStates coralState;

    private ManipulatorSubsystem() {
        super("ManipulatorSubsystem");

        setState("Algae", SubsystemStates.IDLE);
        setState("Coral", SubsystemStates.IDLE);

        CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
        limitConfigs.StatorCurrentLimitEnable = true;

        limitConfigs.StatorCurrentLimit = ManipulatorConstants.CORAL_CURRENT_LIMIT;
        coralMotor.getConfigurator().apply(limitConfigs);

        limitConfigs.StatorCurrentLimit = ManipulatorConstants.ALGAE_CURRENT_LIMIT;
        algaeMotor.getConfigurator().apply(limitConfigs);
    }

    @Override
    public void periodic() {}

    /**
     * Set the speed of the Manipulator Coral motor.
     * @param speed Speed between -1.0 and 1.0
     */
    public void setCoralMotor(double speed) {
        coralMotor.set(speed);
    }

    /**
     * Set the speed of the Manipulator Algae motor.
     * @param speed Speed between -1.0 and 1.0
     */
    public void setAlgaeMotor(double speed) {
        algaeMotor.set(speed);
    }

    /**
     * Logs the state of the subsystem to AdvantageKit and SmartDashboard
     * @param subname Specific name of the part of the Manipulator (eg. Coral, Intake, Algae)
     * @param state Current state to be logged
     */
    public void setState(String subname, SubsystemStates state) {
        Logger.recordOutput("Manipulator/"+subname+"State", state);
        SmartDashboard.putString("Manipulator/"+subname+"State", state.toString());

        if(subname.equals("Coral")) {
            coralState = state;
        }
    }

    public SubsystemStates getCoralState() {
        return coralState;
    }

    /**
     * Gets the active stator current of the Coral motor.
     * @return Current in Amps
     */
    public double getCoralStatorCurrent() {
        return coralMotor.getStatorCurrent().getValueAsDouble();
    }

    /**
     * Gets the active stator current of the Algae motor.
     * @return Current in Amps
     */
    public double getAlgaeStatorCurrent() {
        return algaeMotor.getStatorCurrent().getValueAsDouble();
    }

    /**
     * Gets the active rotor velocity of the Coral motor.
     * @return the velocity
     */
    public double getAlgaeRotorVelocity() {
        return algaeMotor.getRotorVelocity().getValueAsDouble();
    }
}