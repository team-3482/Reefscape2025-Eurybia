// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.auto.PIDAlignReefCommand;
import frc.robot.constants.PhysicalConstants.ElevatorConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.constants.VirtualConstants.*;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.elevator.MoveElevatorCommand;
import frc.robot.elevator.PostIntakeSafetyCommand;
import frc.robot.elevator.ZeroElevatorCommand;
import frc.robot.led.LEDSubsystem;
import frc.robot.manipulator.*;
import frc.robot.pivot.MovePivotCommand;
import frc.robot.pivot.PivotSafetyCommand;
import frc.robot.pivot.PivotSubsystem;
import frc.robot.pivot.ZeroPivotCommand;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.swerve.SwerveTelemetry;
import frc.robot.utilities.CommandGenerators;
import frc.robot.vision.LimelightSubsystem;
import frc.robot.vision.QuestNavSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.Map;
import java.util.function.Supplier;

public class RobotContainer {
    // Use Bill Pugh Singleton Pattern for efficient lazy initialization (thread-safe !)
    private static class RobotContainerHolder {
        private static final RobotContainer INSTANCE = new RobotContainer();
    }

    public static RobotContainer getInstance() {
        return RobotContainerHolder.INSTANCE;
    }

    private final SendableChooser<Command> autoChooser;
    private Command auton = null;

    // Instance of the controllers used to drive the robot
    private final CommandXboxController driverController;
    private final XboxController driverController_HID;
    private final CommandXboxController operatorController;

    public RobotContainer() {
        // Initialize controllers
        this.driverController = new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_ID);
        this.driverController_HID = this.driverController.getHID();
        this.operatorController = new CommandXboxController(ControllerConstants.OPERATOR_CONTROLLER_ID);

        configureDrivetrain();
        initializeSubsystems();

        this.autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be Commands.none()
        this.autoChooser.onChange((Command autoCommand) -> this.auton = autoCommand); // Reloads the stored auto

        SmartDashboard.putData("Auto Chooser", this.autoChooser);
        SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());
    }

    /** Creates instances of each subsystem so periodic runs on startup. */
    @SuppressWarnings("ResultOfMethodCallIgnored")
    private void initializeSubsystems() {
        LEDSubsystem.getInstance();
        LimelightSubsystem.getInstance();
        QuestNavSubsystem.getInstance();
        ElevatorSubsystem.getInstance();
        PivotSubsystem.getInstance();
        ManipulatorSubsystem.getInstance();
    }


    /** Register all NamedCommands for PathPlanner use */
    private void registerNamedCommands() {
        NamedCommands.registerCommand("MoveElevatorToIntake",
            new MoveElevatorCommand(ElevatorPositions.INTAKE, false));
        NamedCommands.registerCommand("MoveElevatorToIdle",
            new MoveElevatorCommand(ElevatorPositions.IDLE_HEIGHT, false));

        NamedCommands.registerCommand("MoveElevatorToL2Coral",
            new MoveElevatorCommand(ElevatorPositions.L2_CORAL, false));
        NamedCommands.registerCommand("MoveElevatorToL3Coral",
            new MoveElevatorCommand(ElevatorPositions.L3_CORAL, false));
        NamedCommands.registerCommand("MoveElevatorToL4Coral",
            new MoveElevatorCommand(ElevatorPositions.L4_CORAL, false));

        NamedCommands.registerCommand("MoveElevatorToL2Algae",
            new MoveElevatorCommand(ElevatorPositions.L2_ALGAE, false));
        NamedCommands.registerCommand("MoveElevatorToL3Algae",
            new MoveElevatorCommand(ElevatorPositions.L3_ALGAE, false));

        NamedCommands.registerCommand("IntakeCoral",
            new IntakeCoralCommand());
        NamedCommands.registerCommand("OuttakeCoral",
            new OuttakeCoralCommand());
        NamedCommands.registerCommand("IntakeAlgae",
            new IntakeAlgaeCommand());
        NamedCommands.registerCommand("OuttakeAlgae",
            new OuttakeAlgaeCommand());

        NamedCommands.registerCommand("ZeroElevator",
            new ZeroElevatorCommand());
        NamedCommands.registerCommand("ZeroPivot",
            new ZeroPivotCommand());

        NamedCommands.registerCommand("PivotSafety",
            new PivotSafetyCommand());
        NamedCommands.registerCommand("PostIntakeSafety",
            new PostIntakeSafetyCommand());

        NamedCommands.registerCommand("PIDAlignLeftReef", CommandGenerators.WaitForLimelightsCommand(
            new PIDAlignReefCommand(-1, false, false)
        ));
        NamedCommands.registerCommand("PIDAlignCenterReef", CommandGenerators.WaitForLimelightsCommand(
            new PIDAlignReefCommand(0, false, false)
        ));
        NamedCommands.registerCommand("PIDAlignRightReef", CommandGenerators.WaitForLimelightsCommand(
            new PIDAlignReefCommand(1, false, false)
        ));
    }

    /**
     * This method initializes the swerve subsystem and configures its bindings with the driver controller.
     * This is based on the Phoenix6 Swerve example.
     */
    private void configureDrivetrain() {
        final SwerveSubsystem Drivetrain = SwerveSubsystem.getInstance();

        final double NormalSpeed = SwerveConstants.kSpeedNormal.in(Units.MetersPerSecond);
        final double MaxSpeed = SwerveConstants.kSpeedAt12Volts.in(Units.MetersPerSecond);
        final double NormalAngularSpeed = SwerveConstants.kAngularSpeedNormal.in(Units.RadiansPerSecond);
        final double FastAngularSpeed = SwerveConstants.kAngularSpeedFast.in(Units.RadiansPerSecond);

        /* Setting up bindings for necessary control of the swerve drive platform */
        final SwerveRequest.FieldCentric fieldCentricDrive_withDeadband = new SwerveRequest
            .FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        final SwerveTelemetry logger = new SwerveTelemetry(MaxSpeed);

        // TODO maybe make these increase/decrease the speed depending on how much the trigger is pressed
        Supplier<Boolean> leftTrigger = () -> this.driverController_HID.getLeftTriggerAxis() >= 0.5;
        Supplier<Boolean> rightTrigger = () -> this.driverController_HID.getRightTriggerAxis() >= 0.5;

        // Drivetrain will execute this command periodically
        Drivetrain.setDefaultCommand(
            Drivetrain.applyRequest(() -> {
                boolean elevatorTooHigh = ElevatorSubsystem.getInstance().getPosition() > ElevatorConstants.SLOW_DRIVE_HEIGHT;
                boolean topSpeed = leftTrigger.get();
                boolean fineControl = rightTrigger.get();

                double linearSpeed = elevatorTooHigh
                    ? (SwerveConstants.kElevatorTooHighSpeed.in(Units.MetersPerSecond))
                    : (topSpeed ? MaxSpeed : NormalSpeed);

                Logger.recordOutput("DriveState/MaxSpeed", linearSpeed);
                SmartDashboard.putNumber("DriveState/MaxSpeed", linearSpeed);

                double angularSpeed = topSpeed ? NormalAngularSpeed : FastAngularSpeed;

                double fineControlMult = fineControl ? ControllerConstants.FINE_CONTROL_MULT : 1;

                return fieldCentricDrive_withDeadband
                    // Drive forward with negative Y (forward)
                    .withVelocityX(-driverController.getLeftY() * linearSpeed * fineControlMult)
                    // Drive left with negative X (left)
                    .withVelocityY(-driverController.getLeftX() * linearSpeed * fineControlMult)
                    // Drive counterclockwise with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * angularSpeed * fineControlMult)

                    .withDeadband(ControllerConstants.DEADBAND * linearSpeed)
                    .withRotationalDeadband(ControllerConstants.DEADBAND * angularSpeed);
            }).ignoringDisable(true)
        );

        // Useful for testing
        // final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
        // this.driverController.y().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(
        //     new Rotation2d(
        //         Math.abs(driverController.getLeftY()) >= 0.25 ? -driverController.getLeftY() : 0,
        //         Math.abs(driverController.getLeftX()) >= 0.25 ? -driverController.getLeftX() : 0
        //     )
        // )));

        // POV / D-PAD
        final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        if (ControllerConstants.DPAD_DRIVE_INPUT) {
            /* POV angle : [X velocity, Y velocity] in m/s */
            final Map<Integer, Integer[]> povSpeeds = Map.ofEntries(
                Map.entry(  0, new Integer[]{ 1,  0}),
                Map.entry( 45, new Integer[]{ 1, -1}),
                Map.entry( 90, new Integer[]{ 0, -1}),
                Map.entry(135, new Integer[]{-1, -1}),
                Map.entry(180, new Integer[]{-1,  0}),
                Map.entry(225, new Integer[]{-1,  1}),
                Map.entry(270, new Integer[]{ 0,  1}),
                Map.entry(315, new Integer[]{ 1,  1})
            );

            povSpeeds.forEach(
                (Integer angle, Integer[] speeds) -> this.driverController.pov(angle).whileTrue(
                    Drivetrain.applyRequest(() -> {
                        boolean faster = leftTrigger.get();
                        boolean robotCentric = rightTrigger.get();

                        return robotCentric
                            ? robotCentricDrive
                            .withVelocityX(speeds[0] * (faster ? 1.5 : 0.25))
                            .withVelocityY(speeds[1] * (faster ? 1.5 : 0.25))
                            : fieldCentricDrive
                            .withVelocityX(speeds[0] * (faster ? 1.5 : 0.25))
                            .withVelocityY(speeds[1] * (faster ? 1.5 : 0.25));
                    })
                )
            );
        }

        Drivetrain.registerTelemetry(logger::telemeterize);
    }

    /** Configures the button bindings of the driver controller. */
    public void configureDriverBindings() {
        // Double Rectangle -> Reset pose
        this.driverController.back().onTrue(Commands.runOnce(() -> SwerveSubsystem.getInstance().resetPose(Pose2d.kZero)));
        // Burger -> Reset rotation to zero
        this.driverController.start().onTrue(Commands.runOnce(() -> SwerveSubsystem.getInstance().seedFieldCentric()));

        // Left Bumper -> Auto-align to left side
        this.driverController.leftBumper().whileTrue(Commands.sequence(
            CommandGenerators.WaitForLimelightsCommand(new PIDAlignReefCommand(-1, true, true)),
            new PIDAlignReefCommand(-1, true, false)
        ));

        // Right Bumper -> Auto-align to right side
        this.driverController.rightBumper().whileTrue(Commands.sequence(
            CommandGenerators.WaitForLimelightsCommand(new PIDAlignReefCommand(1, true, true)),
            new PIDAlignReefCommand(1, true, false)
        ));

        // A -> Auto-align to Algae
        this.driverController.a().whileTrue(
            CommandGenerators.WaitForLimelightsCommand(new PIDAlignReefCommand(0, false, false)));
    }

    /** Configures the button bindings of the operator controller. */
    public void configureOperatorBindings() {
        // B -> Cancel all commands
        this.operatorController.b().onTrue(Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll()));

        Supplier<Boolean> slowElevatorSupplier = () -> this.operatorController.getHID().getRightTriggerAxis() >= 0.5;

        // D-PAD Left -> Zero
        this.operatorController.povLeft().onTrue(Commands.sequence(
            new PivotSafetyCommand(),
            new ZeroElevatorCommand()
        ));

        // D-PAD Down -> L2 Coral
        this.operatorController.povDown().onTrue(Commands.sequence(
            new PostIntakeSafetyCommand(),
            new PivotSafetyCommand(ElevatorPositions.L2_CORAL),
            new MoveElevatorCommand(ElevatorPositions.L2_CORAL, slowElevatorSupplier),
            new MovePivotCommand(PivotPositions.CORAL, PivotPositionNames.CORAL)
        ));

        // D-PAD Right -> L3 Coral
        this.operatorController.povRight().onTrue(Commands.sequence(
            new PostIntakeSafetyCommand(),
            new PivotSafetyCommand(ElevatorPositions.L3_CORAL),
            new MoveElevatorCommand(ElevatorPositions.L3_CORAL, slowElevatorSupplier),
            new MovePivotCommand(PivotPositions.CORAL, PivotPositionNames.CORAL)
        ));


        // D-PAD Up -> L4 Coral
        this.operatorController.povUp().onTrue(Commands.sequence(
            new PostIntakeSafetyCommand(),
            new PivotSafetyCommand(ElevatorPositions.L4_CORAL),
            new MoveElevatorCommand(ElevatorPositions.L4_CORAL, slowElevatorSupplier),
            new MovePivotCommand(PivotPositions.CORAL, PivotPositionNames.CORAL)
        ));

        // TODO PLACEHOLDER!!!!!!!!!
        this.operatorController.leftStick().onTrue(Commands.sequence(
            new PivotSafetyCommand(ElevatorPositions.IDLE_HEIGHT),
            new MoveElevatorCommand(ElevatorPositions.IDLE_HEIGHT, slowElevatorSupplier),
            new MovePivotCommand(PivotPositions.PROCESSOR, PivotPositionNames.CORAL)
        ));

        // TODO PLACEHOLDER!!!!!!!!!
        this.operatorController.rightStick().onTrue(Commands.sequence(
            new PivotSafetyCommand(ElevatorPositions.L4_CORAL),
            new MoveElevatorCommand(ElevatorPositions.L4_CORAL, slowElevatorSupplier),
            new MovePivotCommand(PivotPositions.BARGE, PivotPositionNames.CORAL)
        ));

        // Left Bumper -> Intake Coral
        if(ManipulatorSubsystem.getInstance().getCoralState().equals(SubsystemStates.IDLE)){
            this.operatorController.leftBumper().onTrue(Commands.sequence(
                new PivotSafetyCommand(ElevatorPositions.INTAKE),
                new MoveElevatorCommand(ElevatorPositions.INTAKE, slowElevatorSupplier),
                new MovePivotCommand(PivotPositions.INTAKE, PivotPositionNames.INTAKE),
                new IntakeCoralCommand(),
                new MoveElevatorCommand(ElevatorPositions.SAFE_CORAL, slowElevatorSupplier),
                new MovePivotCommand(PivotPositions.ELEVATING, PivotPositionNames.ELEVATING)
            ));
        }

        //Right Bumper -> Outtake Coral
        this.operatorController.rightBumper().whileTrue(new OuttakeCoralCommand());

        // A -> Intake L2 Algae, Y -> Intake L3 Algae
        this.operatorController.a().onTrue(Commands.sequence(
            new PivotSafetyCommand(),
            new MoveElevatorCommand(ElevatorPositions.L2_ALGAE, slowElevatorSupplier),
            new MovePivotCommand(PivotPositions.ALGAE, PivotPositionNames.ALGAE),
            new IntakeAlgaeCommand()
        ));

        this.operatorController.y().onTrue(Commands.sequence(
            new PivotSafetyCommand(),
            new MoveElevatorCommand(ElevatorPositions.L3_ALGAE, slowElevatorSupplier),
            new MovePivotCommand(PivotPositions.ALGAE, PivotPositionNames.ALGAE),
            new IntakeAlgaeCommand()
        ));

        // X -> Outtake Algae
        this.operatorController.x().whileTrue(new OuttakeAlgaeCommand());


        // Double Rectangle -> Zero Elevator
        // this.operatorController.back().onTrue(Commands.sequence(
        //     new PivotSafetyCommand(),
        //     new ZeroElevatorCommand()
        // ));

        // Burger -> Zero Pivot
        this.operatorController.start().onTrue(new ZeroPivotCommand());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * @return The command to run in autonomous.
     */
    public Command getAutonomousCommand() {
        if (this.auton == null) {
            this.auton = this.autoChooser.getSelected();
        }
        return this.auton;
    }
}