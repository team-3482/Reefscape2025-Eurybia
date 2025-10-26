package frc.robot.manipulator;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.PhysicalConstants.ManipulatorConstants;
import frc.robot.constants.VirtualConstants.SubsystemStates;
import frc.robot.led.LEDSubsystem;
import frc.robot.led.StatusColors;

/** A command that intakes the coral */
public class IntakeCoralCommand extends Command {
    private Debouncer debouncer;

    public IntakeCoralCommand() {
        setName("IntakeCoralCommand");
        addRequirements(ManipulatorSubsystem.getInstance());

        debouncer = new Debouncer(.1);
    }


    @Override
    public void initialize() {
        ManipulatorSubsystem.getInstance().setCoralMotor(-ManipulatorConstants.CORAL_INTAKE_SPEED);

        ManipulatorSubsystem.getInstance().setState("Coral", SubsystemStates.INTAKING);
    }

    @Override
    public void execute() {}


    @Override
    public void end(boolean interrupted) {
        if(!interrupted) {
            LEDSubsystem.getInstance().setColor(StatusColors.CORAL);
        }

        ManipulatorSubsystem.getInstance().setCoralMotor(0);

        ManipulatorSubsystem.getInstance().setState("Coral", SubsystemStates.HOLDING);
    }


    @Override
    public boolean isFinished() {
        return debouncer.calculate(ManipulatorSubsystem.getInstance().getCoralStatorCurrent() >= ManipulatorConstants.CORAL_CURRENT_LIMIT);
    }
}