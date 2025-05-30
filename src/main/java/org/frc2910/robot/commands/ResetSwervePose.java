package org.frc2910.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ResetSwervePose extends SequentialCommandGroup {
    public ResetSwervePose(Command... commands) {
        super(commands);
    }
}
