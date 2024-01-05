package org.firstinspires.ftc.teamcode.common.subsystem;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.SimpleServo;

public class ClawSubsystem extends LinearOpMode{

    SimpleServo clawAngleServo;
    SimpleServo clawRightServo;
    SimpleServo clawLeftServo;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        //myServo = new SimpleServo(hardwareMap, "myServo", 0, 180);


        clawAngleServo = new SimpleServo(
                hardwareMap, "ClawAngleServo", 10, 120,
                AngleUnit.DEGREES
        );

        clawRightServo = new SimpleServo(
                hardwareMap, "ClawRightServo", 10, 120,
                AngleUnit.DEGREES
        );

        clawLeftServo = new SimpleServo(
                hardwareMap, "ClawLeftServo", 10, 120,
                AngleUnit.DEGREES
        );
        waitForStart();


        clawAngleServo.setInverted(true);
        //clawRightServo.setInverted(true);
        //clawLeftServo.setInverted(true);

        while (opModeIsActive()) {

            if (gamepad1.options){
                clawAngleServo.turnToAngle(160);
                telemetry.addData("Hardware: ", clawAngleServo.getAngle());
                telemetry.update();
            }

            if (gamepad1.share){
                clawAngleServo.turnToAngle(80);
                telemetry.addData("Hardware: ", clawAngleServo.getAngle());
                telemetry.update();
            }

            if (gamepad1.guide){
                clawAngleServo.turnToAngle(70);
                telemetry.addData("Hardware: ", clawAngleServo.getAngle());
                telemetry.update();
            }

            if (gamepad1.dpad_up){
                clawRightServo.turnToAngle(80);
                clawLeftServo.turnToAngle(45);
                telemetry.addData("Hardware: ", clawRightServo.getAngle());
                telemetry.addData("Hardware: ", clawLeftServo.getAngle());
                telemetry.update();
            }

            if (gamepad1.dpad_down){
                clawRightServo.turnToAngle(40);
                clawLeftServo.turnToAngle(80);
                telemetry.addData("Hardware: ", clawRightServo.getAngle());
                telemetry.addData("Hardware: ", clawLeftServo.getAngle());
                telemetry.update();
            }

        }
    }}
