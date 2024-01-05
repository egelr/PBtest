package org.firstinspires.ftc.teamcode.opmode.teleop;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.centerstage.ClawSide;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.IntakeCommand;
//import org.firstinspires.ftc.teamcode.common.commandbase.subsystemcommand.ScoreCommand;
import org.firstinspires.ftc.teamcode.common.drive.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;
//import org.firstinspires.ftc.teamcode.common.hardware.Globals;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.ClawSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystem.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.common.util.InverseKinematics;
import org.firstinspires.ftc.teamcode.common.util.MathUtils;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WServo;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WServo2;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WEncoder;

@TeleOp(name = "OpMode")
public class OpMode extends CommandOpMode {
    private final RobotHardware robot = RobotHardware.getInstance();
    private MecanumDrivetrain drivetrain;
    private IntakeSubsystem intake;
    private ExtensionSubsystem extension;
    private double loopTime = 0.0;
    public static double targetpos = 0;
    public int height = 0;

    private ClawSubsystem claw;

    private GamepadEx gamepadEx;

    private boolean lastJoystickUp = false;
    private boolean lastJoystickDown = false;
    private final boolean rightStickGreat = false;
    private boolean lastRightStickGreat = false;
    public WServo clawAngleServo;
    public WServo clawLeftServo;
    public WServo clawRightServo;
    public WServo2 clawAngleServo1;
    public WServo2 clawLeftServo1;
    public WServo2 clawRightServo1;

    public WEncoder m_motor;
    public WEncoder m_motor_2;
    public double sp;
    @Override
    public void initialize() {
        GamepadEx driverOp = new GamepadEx(gamepad1);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap, telemetry);
        drivetrain = new MecanumDrivetrain();
        claw = new ClawSubsystem();
        intake = new IntakeSubsystem();
        robot.addSubsystem(drivetrain, claw, intake);

        gamepadEx = new GamepadEx(gamepad1);
        //robot.intakePivotActuator.setTargetPosition(targetpos);
        //robot.intakePivotActuator.write();

        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new ConditionalCommand(
                        new IntakeCommand(intake, IntakeSubsystem.ClawState.INTERMEDIATE, ClawSide.LEFT),
                        new IntakeCommand(intake, IntakeSubsystem.ClawState.OPEN, ClawSide.LEFT),
                        () -> (intake.leftClaw == (IntakeSubsystem.ClawState.CLOSED))));
        gamepadEx.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new ConditionalCommand(
                        new IntakeCommand(intake, IntakeSubsystem.ClawState.INTERMEDIATE, ClawSide.RIGHT),
                        new IntakeCommand(intake, IntakeSubsystem.ClawState.OPEN, ClawSide.RIGHT),
                        () -> (intake.leftClaw == (IntakeSubsystem.ClawState.CLOSED))));

        robot.read();

        while (opModeIsActive()) {

            if (gamepad1.right_bumper) {
                sp = 0.006;
                m_motor.setTargetPosition(1540);
            }
            if (gamepad1.left_bumper) {
                sp = 0.01;
                m_motor.setTargetPosition(-1450);
            }

            while (!m_motor.atTargetPosition()) {
                m_motor.set(sp);
                telemetry.addData("Status ", m_motor.getCurrentPosition());
                telemetry.update();

            }



            if (gamepad1.square) {
                //      sp = 0.02;
                m_motor_2.setTargetPosition(800);

                telemetry.addData("share", "lllll");
                telemetry.addData("Status ", m_motor.getCurrentPosition());
                telemetry.update();
            }

            if (gamepad1.triangle) {
                //    sp = 0.02;
                m_motor_2.setTargetPosition(0);
                telemetry.addData("options", "llll");
                telemetry.addData("Status ", m_motor.getCurrentPosition());
                telemetry.update();
                //sp = 3;
                //m_motor_2.setRunMode(Motor.RunMode.RawPower);

            }


            while (!m_motor_2.atTargetPosition()) {
                m_motor_2.set(0.2);
                telemetry.addData("Status", "Moving to target position");
                telemetry.addData("Current Position", m_motor_2.getCurrentPosition());
                //    telemetry.addData("Target Position", targetPos_2);
                telemetry.update();

            }


            m_motor.stopMotor(); // stop the motor
            m_motor_2.stopMotor(); // stop the motor
            //m_motor_2.stopMotor();
            sleep(10);
        }
    }

    @Override
    public void run() {
        robot.read();

        drivetrain.set(new Pose(gamepad1.left_stick_x, -gamepad1.left_stick_y, MathUtils.joystickScalar(-gamepad1.left_trigger + gamepad1.right_trigger, 0.01)), 0);

        boolean currentJoystickUp = gamepad2.right_stick_y < -0.5;
        boolean currentJoystickDown = gamepad2.right_stick_y > 0.5;
        /*
        while (!isStopRequested()) {
            if (gamepad1.options){
                clawAngleServo1.turnToAngle(160);
                telemetry.addData("Hardware: ", clawAngleServo1.getAngle());
                telemetry.update();
            }

            if (gamepad1.share){
                clawAngleServo1.turnToAngle(80);
                telemetry.addData("Hardware: ", clawAngleServo1.getAngle());
                telemetry.update();
            }

            if (gamepad1.guide){
                clawAngleServo1.turnToAngle(70);
                telemetry.addData("Hardware: ", clawAngleServo1.getAngle());
                telemetry.update();
            }

            if (gamepad1.dpad_up){
                clawRightServo1.turnToAngle(80);
                clawLeftServo1.turnToAngle(45);
                telemetry.addData("Hardware: ", clawRightServo1.getAngle());
                telemetry.addData("Hardware: ", clawLeftServo1.getAngle());
                telemetry.update();
            }

            if (gamepad1.dpad_down){
                clawRightServo1.turnToAngle(40);
                clawLeftServo1.turnToAngle(80);
                telemetry.addData("Hardware: ", clawRightServo1.getAngle());
                telemetry.addData("Hardware: ", clawLeftServo1.getAngle());
                telemetry.update();
            }

        }

         */
        if (currentJoystickUp && !lastJoystickUp) {
            // height go upp
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            /*
                            new InstantCommand(() -> extension.incrementBackdropHeight(1)),
                            new ConditionalCommand(
                                    new ScoreCommand(robot, 3, extension.getBackdropHeight()),
                                    new WaitCommand(1),
                                    () -> extension.getScoring()
                            )

                             */
                    ));
        }

        if (currentJoystickDown && !lastJoystickDown) {
            // gheight go dwodn
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            //new InstantCommand(() -> extension.incrementBackdropHeight(-1)),
                            //new InstantCommand(() -> InverseKinematics.calculateTarget(3, extension.getBackdropHeight())),
                            //new ConditionalCommand(
                                    //new ScoreCommand(robot, 3, extension.getBackdropHeight()),
                                    //new WaitCommand(1),
                                    //() -> extension.getScoring()
                            //)
                    ));
        }
        lastJoystickUp = currentJoystickUp;
        lastJoystickDown = currentJoystickDown;

        // input
        super.run();
        robot.periodic();

        lastRightStickGreat = rightStickGreat;

        //telemetry.addData("extension", robot.extensionActuator.getPosition());
        //telemetry.addData("angle", robot.pitchActuator.getPosition());
        //telemetry.addData("LEVEL", extension.getBackdropHeight());
        //telemetry.addData("Textension", InverseKinematics.t_extension);
        //telemetry.addData("Tangle", InverseKinematics.t_angle);

//        telemetry.addData("targetAngle", extension.t_angle);
//        telemetry.addData("targetExtension", robot.extensionActuator.getTargetPosition());
//        telemetry.addData("diffX", extension.diff_x);
//        telemetry.addData("diffy", extension.diff_y);
//        telemetry.addData("velocity", localizer.getNewPoseVelocity());
        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loopTime));
        loopTime = loop;
        telemetry.update();
        robot.write();
        robot.clearBulkCache();


    }

}
