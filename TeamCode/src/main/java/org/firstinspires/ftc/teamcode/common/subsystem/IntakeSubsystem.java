package org.firstinspires.ftc.teamcode.common.subsystem;
/*
import org.firstinspires.ftc.teamcode.common.centerstage.ClawSide;
import org.firstinspires.ftc.teamcode.common.centerstage.Side;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.util.MathUtils;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WSubsystem;
import org.jetbrains.annotations.NotNull;
 */
import org.firstinspires.ftc.teamcode.common.centerstage.ClawSide;
import org.firstinspires.ftc.teamcode.common.centerstage.Side;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.util.MathUtils;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WSubsystem;
import org.jetbrains.annotations.NotNull;
public class IntakeSubsystem {
    private final RobotHardware robot;

    //    private ClawState clawState;
    private PivotState pivotState;

    public ClawState leftClaw = ClawState.CLOSED;
    public ClawState rightClaw = ClawState.CLOSED;

    private boolean pixelLeftTop,    pixelRightTop,
            pixelLeftBottom, pixelRightBottom;

    public enum ClawState {
        CLOSED,
        INTERMEDIATE,
        OPEN
    }
    public IntakeSubsystem() {
        this.robot = RobotHardware.getInstance();
//        this.clawState = new HashMap<>();
//        clawState.put(ClawSide.LEFT, ClawState.CLOSED);
//        clawState.put(ClawSide.RIGHT, ClawState.CLOSED);

        updateState(ClawState.CLOSED, ClawSide.BOTH);
    }

    public void updateState(@NotNull ClawState state, @NotNull ClawSide side) {
        double position = getClawStatePosition(state, side);
//        this.clawState = state;
        switch(side) {
            case LEFT:
                robot.clawLeftServo.setPosition(position);
                this.leftClaw = state;
//                this.clawState.replace(side, state);
                break;
            case RIGHT:
                robot.clawRightServo.setPosition(position);
                this.rightClaw = state;
//                this.clawState.replace(side, state);
                break;
            case BOTH:
                position = clawStatePosition(state, ClawSide.LEFT);
                robot.clawLeftServo.setPosition(position);
                this.leftClaw = state;
                position = getClawStatePosition(state, ClawSide.RIGHT);
                robot.clawRightServo.setPosition(position);
                this.rightClaw = state;
//                this.clawState.replace(ClawSide.LEFT, state);
//                this.clawState.replace(ClawSide.RIGHT, state);

                break;
        }
    }



}
