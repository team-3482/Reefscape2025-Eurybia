// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/** Contains virtual constants that do not correlate to anything physical in real life. */
public final class VirtualConstants {
    /** Constants for the controller and any controller-related assignments. */
    public static final class ControllerConstants {
        /** DriverStation ID of the driver controller. */
        public static final int DRIVER_CONTROLLER_ID = 0;
        /** DriverStation ID of the operator controller. */
        public static final int OPERATOR_CONTROLLER_ID = 1;
        /** Removes input around the joystick's center (eliminates stick drift). */
        public static final double DEADBAND = 0.075;
        /** Whether to accept directional pad input for movement. */
        public static final boolean DPAD_DRIVE_INPUT = true;
        /** Speed multiplier when using fine-control movement. */
        public static final double FINE_CONTROL_MULT = 0.15;
    }

    public enum SubsystemStates {
        IDLE,
        INTAKING,
        OUTTAKING,
        HOLDING
    }

    public enum PivotPositionNames {
        INTAKE,
        CORAL,
        ALGAE,
        ELEVATING
    }

    /** Constants for elevator heights */
    public static final class ElevatorPositions {
        public static final double INTAKE = 0.05;
        public static final double L2_CORAL = 0.19;
        public static final double L3_CORAL = 0.38;
        public static final double L4_CORAL = 0.78;

        public static final double L2_ALGAE = 0.14;
        public static final double L3_ALGAE = 0.41;

        public static final double IDLE_HEIGHT = INTAKE;

        public static final double SAFE_CORAL = 0.05;
    }

    public static final class PivotPositions {
        public static final double CORAL = 160;
        public static final double ALGAE = 90;
        public static final double PROCESSOR = 45;
        public static final double BARGE = 135;
        public static final double INTAKE = 0;
        public static final double ELEVATING = ALGAE;
    }
}