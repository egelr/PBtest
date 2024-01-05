package org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.centerstage.ClawSide;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;
public class IntakeCommand extends InstantCommand {
    public IntakeCommand(IntakeSubsystem intake, IntakeSubsystem.ClawState state, ClawSide side) {
        super(() -> intake.updateState(state, side)
        );

    }
}
