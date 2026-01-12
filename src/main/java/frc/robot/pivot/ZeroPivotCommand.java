package frc.robot.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.PhysicalConstants.PivotConstants;
import frc.robot.led.LEDSubsystem;
import frc.robot.led.StatusColors;

/** A command that pushes the pivot down to the hardstop to zero it */
public class ZeroPivotCommand extends Command {

    /** Creates a new ZeroPivotCommand. */
    public ZeroPivotCommand() {
        setName("ZeroPivotCommand");

        addRequirements(PivotSubsystem.getInstance());
    }


    @Override
    public void initialize() {
        System.out.println("zeroing pivot");
        PivotSubsystem.getInstance().setPivotSpeed(-PivotConstants.ZERO_SPEED, false);
    }


    @Override
    public void execute() {}


    @Override
    public void end(boolean interrupted) {
        PivotSubsystem.getInstance().setPivotSpeed(0);
        PivotSubsystem.getInstance().setPositionHardstop();

        LEDSubsystem.getInstance().setColor(StatusColors.OK);

        System.out.println("Pivot Zeroed!");
    }

    @Override
    public boolean isFinished() {
        return PivotSubsystem.getInstance().isAtEndstop();
    }
}