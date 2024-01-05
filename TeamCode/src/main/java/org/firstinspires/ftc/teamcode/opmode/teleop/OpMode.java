package org.firstinspires.ftc.teamcode.opmode.teleop;
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//import org.firstinspires.ftc.teamcode.common.centerstage.ClawSide;
//import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ClawCommand;
//import org.firstinspires.ftc.teamcode.common.commandbase.subsytemcommand.ScoreCommand;
import org.firstinspires.ftc.teamcode.common.drive.drivetrain.MecanumDrivetrain;
//import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;
//import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.ClawSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.common.util.InverseKinematics;
//import org.firstinspires.ftc.teamcode.common.util.MathUtils;
@TeleOp(name = "OpMode")
public class OpMode extends CommandOpMode {
    private final RobotHardware robot = RobotHardware.getInstance();
    private MecanumDrivetrain drivetrain;
    private IntakeSubsystem intake;

    private ClawSubsystem claw;

    private GamepadEx gamepadEx;
    @Override
    public void initialize() {
        GamepadEx driverOp = new GamepadEx(gamepad1);
        //telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap, telemetry);
        drivetrain = new MecanumDrivetrain();
        claw = new ClawSubsystem();
        intake = new IntakeSubsystem();
        robot.addSubsystem(drivetrain, claw, intake);

        gamepadEx = new GamepadEx(gamepad1);

        gamepadEx.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new ConditionalCommand(
                        clawAngleServo.turnToAngle(160),
                        telemetry.addData("Hardware: ", clawAngleServo.getAngle()),
                        telemetry.update()

    }
}
