package frc.robot.utilities;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.vision.LimelightSubsystem;
import org.littletonrobotics.junction.Logger;

import java.text.DecimalFormat;

import static edu.wpi.first.units.Units.Meters;

public class CommandGenerators {
    /**
     * A command that waits for limelights to be close to odometry before finishing.
     * @param The PIDAlign Command to run after waiting.
     * @return The command.
     */
    public static Command WaitForLimelightsCommand(Command PIDAlignCommand) {
        final DecimalFormat DOUBLE_FORMAT = new DecimalFormat("#.##");
        final Timer timeoutTimer = new Timer();

        return Commands.sequence(
            Commands.runOnce(() -> timeoutTimer.restart()),

            Commands.waitUntil(() -> {
                LimelightSubsystem.getInstance().waitingForLimelights = true;

                if (
                    timeoutTimer.hasElapsed(DriverStation.isTeleop() ? 0.5 : 1)
                        || SwerveSubsystem.getInstance().getDistance(
                        LimelightSubsystem.getInstance().getPose2d().pose2d.getTranslation()
                    ).in(Meters) <= 0.1
                ) {
                    String wastedTime = DOUBLE_FORMAT.format(timeoutTimer.get());
                    Logger.recordOutput("PIDAlign Wasted Time", wastedTime);

                    String wastedTimeMsg = "Wasted "
                        + wastedTime
                        + " seconds waiting for Limelights";
                    System.out.println(wastedTimeMsg);
                    // Elastic.sendNotification(new Notification(NotificationLevel.WARNING, "PIDAlign",  wastedTimeMsg));

                    LimelightSubsystem.getInstance().waitingForLimelights = false;
                    return true;
                }
                else {
                    return false;
                }
            }),

            PIDAlignCommand
        );
    }
}