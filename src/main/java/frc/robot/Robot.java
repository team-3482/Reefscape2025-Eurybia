// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.VirtualConstants.SubsystemStates;
import frc.robot.led.LEDSubsystem;
import frc.robot.led.StatusColors;
import frc.robot.manipulator.ManipulatorSubsystem;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import java.io.File;

public class Robot extends LoggedRobot {
    private Command auton;

    public Robot() {
        RobotContainer robotContainer = RobotContainer.getInstance();

        robotContainer.configureDriverBindings();
        robotContainer.configureOperatorBindings();

        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

        // Logging
        Logger.recordMetadata("ProjectName", "Reefscape2025-Offseason");

        if (isReal()) {
            // Random parent directory name to differentiate logs.
            // I tried to use the date & time, but the RIO doesn't have an accurate timestamp since there's no RTC.
            String path = "/U/logs/" + (long)(Math.random() * Math.pow(10, 16));

            System.out.println("logging to: " + path + " (new directory: " + new File(path).mkdirs() + ")");

            // SignalLogger.setPath(path);

            Logger.addDataReceiver(new WPILOGWriter(path)); // Log to a USB stick ("/U/logs")
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables

            Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
            // This will always be either 0 or 1, so the > sign is used to suppress the comparing identical expressions.
            //noinspection ConstantValue (IntelliJ warning suppression)
            Logger.recordMetadata("GitDirty", BuildConstants.DIRTY > 0 ? "true" : "false");
            Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);

            Logger.start();
        }

        FollowPathCommand.warmupCommand().schedule();
        // Eager-load the auton command so it's ready right away
        RobotContainer.getInstance().getAutonomousCommand();

        // Solid orange like the RSL when disabled
        LEDSubsystem.getInstance().setColor(StatusColors.RSL);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        double voltage = RobotController.getBatteryVoltage();
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
        SmartDashboard.putNumber("Voltage", voltage);

        Logger.recordOutput("Voltage", voltage);
    }

    @Override
    public void disabledInit() {
        // Solid orange like the RSL when disabled
        LEDSubsystem.getInstance().setColor(StatusColors.RSL);
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {
        LEDSubsystem.getInstance().setColor(StatusColors.OFF);
    }

    @Override
    public void autonomousInit() {
        this.auton = RobotContainer.getInstance().getAutonomousCommand();

        Logger.recordOutput("Auton/AutonCommand", auton.getName());

        if (this.auton != null) {
            ManipulatorSubsystem.getInstance().setState("Coral", SubsystemStates.HOLDING);
            this.auton.schedule();
        } else {
            System.err.println("No auton command found.");
            LEDSubsystem.getInstance().setColor(StatusColors.ERROR);
        }

        // Elastic.selectTab(VirtualConstants.DashboardTabNames.AUTON); // this lags the bot after enabling so it's commented out
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        if (auton != null) {
            this.auton.cancel();
            LEDSubsystem.getInstance().setColor(StatusColors.OK);
        }

        // Elastic.selectTab(VirtualConstants.DashboardTabNames.TELEOP);
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {}

    @Override
    public void testPeriodic() {}

    @Override
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {}
}