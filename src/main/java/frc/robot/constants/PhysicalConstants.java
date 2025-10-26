package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.util.Units;

/** Contains physical constants that are related to real-life things.
 * For example, device IDs, speeds, ratios/multipliers, etc.
 * */
public final class PhysicalConstants {
    public static final class RobotConstants {
        /** CAN Bus for all CTRE devices */
        public static final CANBus CAN_BUS = new CANBus("ctre");
    }

    /** Constants for the LED strips */
    public static final class LEDConstants {
        /** Id for PWM */
        public static final int PWM_HEADER = 0;
        /** This is how many LEDs are on the strip */
        public static final int LED_LENGTH = 80;
        /** Blink cooldown; time that the selected blink color will stay for, then blink */
        public static final double BLINK_COOLDOWN = 0.2;
    }

    public static final class ElevatorConstants {
        /** The CAN ID for the Left Elevator TalonFX */
        public static final int LEFT_MOTOR_ID = 20;
        /** The CAN ID for the Right Elevator TalonFX */
        public static final int RIGHT_MOTOR_ID = 21;

        /**
         * The diameter of the bar that the string wraps around on the elevator
         * @implSpec This is in meters and is very important to calculate the position of the elevator
         */
        public static final double ROLLER_DIAMETER = Units.inchesToMeters(1.125);

        /** Heuristic offset multiplier if math is wrong */
        public static final double LINEAR_CONSTANT_MULT = 0.75;

        /** Tolerance used for elevator command in meters. */
        public static final double HEIGHT_TOLERANCE = 0.05;

        /** The voltage to run the motors at to zero the elevator */
        public static final double ZERO_ELEVATOR_VOLTAGE = 2;

        /** Elevator Motor Stator Current Limit */
        public static final double STATOR_CURRENT_LIMIT = 80;

        /** The maximum height that the elevator can go to without the pivot being in the way */
        public static final double SAFE_PIVOT_HEIGHT = 0.35;

        /** The maximum height the elevator can raise to before it slows the drivetrain to prevent tipping over */
        public static final double SLOW_DRIVE_HEIGHT = 0.4;

        /** The maximum height of the elevator */
        public static final double MAX_HEIGHT = 0.78;

        // TODO tune MM
        /* Motion Magic Config */
        public static final double ROTOR_TO_MECHANISM_RATIO = 3; // 36:12, 3:1
        public static final double CRUISE_SPEED = 30;
        public static final double SLOW_CRUISE_SPEED = 10;
        public static final double ACCELERATION = 60;
        public static final double SLOW_ACCELERATION = 30;

        /** Gains used for Motion Magic slot 0. */
        public static final class Slot0Gains {
            public static final double kG = 0.37;
            public static final double kS = 0;
            public static final double kV = 0;
            public static final double kA = 0;
            public static final double kP = 16;
            public static final double kI = 0;
            public static final double kD = 0;
        }
    }

    public static final class PivotConstants {
        /** The CAN ID for the Pivot TalonFX */
        public static final int MOTOR_ID = 22;

        /** The DIO ID for the limit switch */
        public static final int LIMIT_SWITCH_ID = 0;

        /** Lower angle limit */
        public static final double LOWER_ANGLE_LIMIT = 0;
        /** Upper angle limit */
        public static final double UPPER_ANGLE_LIMIT = 173;

        /** Tolerance for Commands using MotionMagic in degrees. */
        public static final double POSITION_TOLERANCE = 2;

        // this could be better but it's good enough and I dont have time anymore
        /* Motion Magic Config */
        public static final double ROTOR_TO_MECHANISM_RATIO = (double) 203280 / 3240;
        public static final double CRUISE_SPEED = 3;
        public static final double ACCELERATION = 7.5;

        /** Gains used for Motion Magic slot 0. */
        public static final class Slot0Gains {
            public static final double kG = 0.15;
            public static final double kS = 0.23;
            public static final double kV = 0;
            public static final double kA = 0;
            public static final double kP = 192;
            public static final double kI = 0;
            public static final double kD = 0;
        }

        /** Minimum safe position to prevent the pivot from crashing when elevator moves */
        public static final double MINIMUM_SAFE = 40;
        /** Maximum safe position to prevent the pivot from crashing when elevator moves */
        public static final double MAXIMUM_SAFE = 140;

        /** The speed that the pivot zeros at */
        public static final double ZERO_SPEED = 0.4;

    }

    /** Constants for the manipulator subsystem */
    public static final class ManipulatorConstants {
        // TODO: Fill out speeds and current limits
        /** The CAN ID for the Manipulator Coral TalonFX */
        public static final int CORAL_MOTOR_ID = 23;
        /** The CAN ID for the Manipulator Algae TalonFX  */
        public static final int ALGAE_MOTOR_ID = 24;

        /** The speed at which the Manipulator Coral motor will intake at */
        public static final double CORAL_INTAKE_SPEED = 0.5;
        /** The speed at which the Manipulator Coral motor will outtake at */
        public static final double CORAL_OUTTAKE_SPEED = 0.8;
        /** The speed at which the Manipulator Algae motor will intake at */
        public static final double ALGAE_INTAKE_SPEED = 0.5;
        /** The speed at which the Manipulator Algae motor will outtake at */
        public static final double ALGAE_OUTTAKE_SPEED = 1;
        /** The speed at which the Manipulator Algae motor will hold the Algae in place with */
        public static final double ALGAE_HOLD_SPEED = 0.15;

        /** The current limit for the Manipulator Coral motor */
        public static final double CORAL_CURRENT_LIMIT = 60;

        /** The current limit for the Manipulator Algae motor */
        public static final double ALGAE_CURRENT_LIMIT = 30;
    }
}