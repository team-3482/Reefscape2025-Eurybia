// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.swerve.SwerveSubsystem;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

import frc.robot.constants.VisionConstants.QuestNavConstants;
import org.littletonrobotics.junction.Logger;

/** Subsystem to manage all QuestNav related logic */
public class QuestNavSubsystem extends SubsystemBase {
    // Use Bill Pugh Singleton Pattern for efficient lazy initialization (thread-safe !)
    private static class QuestNavSubsystemHolder {
        private static final QuestNavSubsystem INSTANCE = new QuestNavSubsystem();
    }

    /** Always use this method to get the singleton instance of this subsystem. */
    public static QuestNavSubsystem getInstance() {
        return QuestNavSubsystemHolder.INSTANCE;
    }

    QuestNav questNav = new QuestNav();
    PoseFrame[] poseFrames;

    private QuestNavSubsystem() {
        super("QuestNavSubsystem");
    }

    @Override
    public void periodic() {
        // Required to run here for QuestNav to work properly
        questNav.commandPeriodic();

        Logger.recordOutput("QuestNav/Tracking", questNav.isTracking());

        if(questNav.isTracking()){
            // Get the latest pose data frames from the Quest
            poseFrames = questNav.getAllUnreadPoseFrames();

            updateSwervePoseEstimation();

            SmartDashboard.putNumber("QuestNav/Latency", questNav.getLatency());
            SmartDashboard.putNumber("QuestNav/FramesPerRobotCycle", poseFrames.length);
            SmartDashboard.putNumber("QuestNav/BatteryPercent", questNav.getBatteryPercent().getAsInt());

            Logger.recordOutput("QuestNav/Latency", questNav.getLatency());
            Logger.recordOutput("QuestNav/FramesPerRobotCycle", poseFrames.length);
            Logger.recordOutput("QuestNav/BatteryPercent", questNav.getBatteryPercent().getAsInt());
        }

    }

    /**
     * Get the latest QuestNav pose
     * @return The QuestNav Pose2d
     */
    public Pose2d getPose() {
        if (poseFrames.length > 0) {
            // Get the most recent Quest pose
            Pose2d questPose = poseFrames[poseFrames.length - 1].questPose();

            // Transform by the mount pose to get your robot pose
            return questPose.transformBy(QuestNavConstants.ROBOT_TO_QUEST.inverse());
        }

        return null;
    }

    /**
     * Tells QuestNav an accurate pose to start tracking from.
     * Will run at beginning of match with AprilTag pose data
     * or later if necessary to recalibrate the Quest.
     * @param actualPose The trusted and accurate pose.
     */
    public void resetPose(Pose2d actualPose){
        questNav.setPose(actualPose.transformBy(QuestNavConstants.ROBOT_TO_QUEST));
    }

    private void updateSwervePoseEstimation() {
        for (PoseFrame poseFrame : poseFrames) {
            double timestamp = poseFrame.dataTimestamp();

            // Transform by the mount pose to get your robot pose
            Pose2d robotPose = poseFrame.questPose().transformBy(QuestNavConstants.ROBOT_TO_QUEST.inverse());

            // QuestNav docs mention that you can optionally apply filtering.
            // Let's test it without filtering first and see if it's necessary.

            // Add the measurement to our estimator
            SwerveSubsystem.getInstance().addVisionMeasurement(robotPose, timestamp, QuestNavConstants.TRUST_STD_DEVS);
        }
    }
}