package frc.robot.manipulator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.PhysicalConstants.ManipulatorConstants;
import frc.robot.constants.VirtualConstants.PivotPositionNames;
import frc.robot.constants.VirtualConstants.SubsystemStates;
import frc.robot.pivot.PivotSubsystem;

/** A command that outtakes the coral */
public class OuttakeCoralCommand extends Command {
    public OuttakeCoralCommand() {
        setName("OuttakeCoralCommand");
        addRequirements(ManipulatorSubsystem.getInstance());
    }


    @Override
    public void initialize() {
        if(PivotSubsystem.getInstance().getPositionName().equals(PivotPositionNames.CORAL)) {
            ManipulatorSubsystem.getInstance().setCoralMotor(ManipulatorConstants.CORAL_OUTTAKE_SPEED);
            ManipulatorSubsystem.getInstance().setState("Coral", SubsystemStates.OUTTAKING);
        }
    }

    @Override
    public void execute() {}


    @Override
    public void end(boolean interrupted) {
        ManipulatorSubsystem.getInstance().setCoralMotor(0);
        ManipulatorSubsystem.getInstance().setState("Coral", SubsystemStates.IDLE);
    }


    @Override
    public boolean isFinished() {
        return false;
    }
}