package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.SimpleServo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.arcrobotics.ftclib.hardware.motors.Motor;


@TeleOp(name = "OpMode")
public class OpMode extends LinearOpMode {
    static final boolean FIELD_CENTRIC = false;
    private DcMotor armLiftMotor;
    private DcMotor armSlideMotor;
    static final double MOTOR_TICK_COUNT = 8192;
    SimpleServo clawAngleServo;
    SimpleServo clawRightServo;
    SimpleServo clawLeftServo;
    public double drive_speed = 0.85;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(
                new Motor(hardwareMap, "dtFrontLeftMotor", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "dtFrontRightMotor", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "dtBackLeftMotor", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "dtBackRightMotor", Motor.GoBILDA.RPM_312)
        );
        int quarterTurn = 8192;

        Motor m_motor = new Motor(hardwareMap, "armLiftMotor", 8192, 60);
        Motor m_motor_2 = new Motor(hardwareMap, "armSlideMotor", 1425.2, 117);
        telemetry.addData("Hardware: ", "Initialized");
        telemetry.update();
        m_motor.setRunMode(Motor.RunMode.PositionControl);
        m_motor_2.setRunMode(Motor.RunMode.PositionControl);
        m_motor.setPositionCoefficient(0.05);
        //double kP = m_motor.getPositionCoefficient();
        m_motor_2.setPositionCoefficient(0.05);
        //double kP_2 = m_motor_2.getPositionCoefficient();

        int pos = m_motor.getCurrentPosition();
        m_motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        m_motor_2.setInverted(true);
        //m_motor_2.resetEncoder();
        m_motor_2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        //m_motor_2.set(0);
        m_motor.setPositionTolerance(50);
        m_motor_2.setPositionTolerance(50);
        double sp = 0.05;
        double sp2 = 0.05;
        int pos2 = m_motor_2.getCurrentPosition();

        clawAngleServo = new SimpleServo(
                hardwareMap, "clawAngleServo", 10, 120,
                AngleUnit.DEGREES
        );

        clawRightServo = new SimpleServo(
                hardwareMap, "clawRightServo", 10, 120,
                AngleUnit.DEGREES
        );

        clawLeftServo = new SimpleServo(
                hardwareMap, "clawLeftServo", 10, 120,
                AngleUnit.DEGREES
        );


        //GamepadEx driverOp = new GamepadEx(gamepad1);
        waitForStart();

        clawAngleServo.setInverted(true);

        while (!isStopRequested()) {

            drive.driveRobotCentric(
                    gamepad1.left_stick_x * drive_speed,
                    -gamepad1.left_stick_y * drive_speed,
                    gamepad1.right_stick_x * drive_speed,
                    false

            );

            if (gamepad1.right_trigger > 0.5) {
                drive_speed = 0.2;
            } else {
                drive_speed = 0.85;
            }

            if (gamepad1.options) {
                clawAngleServo.turnToAngle(160);
                telemetry.addData("Hardware: ", clawAngleServo.getAngle());
                telemetry.update();
            }

            if (gamepad1.share) {
                clawAngleServo.turnToAngle(80);
                telemetry.addData("Hardware: ", clawAngleServo.getAngle());
                telemetry.update();
            }

            if (gamepad1.guide) {
                clawAngleServo.turnToAngle(70);
                telemetry.addData("Hardware: ", clawAngleServo.getAngle());
                telemetry.update();
            }

            if (gamepad1.dpad_up) {
                clawRightServo.turnToAngle(80);
                clawLeftServo.turnToAngle(45);
                telemetry.addData("Hardware: ", clawRightServo.getAngle());
                telemetry.addData("Hardware: ", clawLeftServo.getAngle());
                telemetry.update();
            }

            if (gamepad1.dpad_down) {
                clawRightServo.turnToAngle(40);
                clawLeftServo.turnToAngle(80);
                telemetry.addData("Hardware: ", clawRightServo.getAngle());
                telemetry.addData("Hardware: ", clawLeftServo.getAngle());
                telemetry.update();
            }
            if (gamepad1.right_bumper) {
                sp = 0.05;
                m_motor.setTargetPosition(pos-2500);
            }
            if (gamepad1.left_bumper) {
                sp = 0.01;
                m_motor.setTargetPosition(pos);
            }

            while (!m_motor.atTargetPosition()) {
                m_motor.set(sp);
                telemetry.addData("Status ", m_motor.getCurrentPosition());
                telemetry.update();

            }


            if (gamepad1.square) {
                sp2 = 0.1;
                m_motor_2.setTargetPosition(pos2);

            }

            if (gamepad1.triangle) {
                sp2 = 0.1;
                m_motor_2.setTargetPosition(pos2+1200);

            }


            while (!m_motor_2.atTargetPosition()) {
                m_motor_2.set(sp2);
            }

            telemetry.addData("Motor 1 ", m_motor.getCurrentPosition());
            telemetry.addData("Motor 2 ", m_motor_2.getCurrentPosition());
            telemetry.update();


            m_motor.stopMotor();
            m_motor_2.stopMotor();
            //sleep(10);


        }


    }

}
