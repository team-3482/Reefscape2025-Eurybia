// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.VirtualConstants.ElevatorPositions;

/** An example command that does nothing. */
public class PostIntakeSafetyCommand extends Command {
    public PostIntakeSafetyCommand() {
        setName("PostIntakeSafetyCommand");
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(ElevatorSubsystem.getInstance());
    }

    @Override
    public void initialize() {
        if(ElevatorSubsystem.getInstance().getPosition() < ElevatorPositions.SAFE_CORAL){
            ElevatorSubsystem.getInstance().motionMagicPosition(ElevatorPositions.SAFE_CORAL, true, false);
        }
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return ElevatorSubsystem.getInstance().withinTolerance(ElevatorPositions.SAFE_CORAL);
    }
}