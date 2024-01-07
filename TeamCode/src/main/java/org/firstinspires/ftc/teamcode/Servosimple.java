package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
@TeleOp
public class Servosimple extends LinearOpMode {
    SimpleServo clawAngleServo;
    SimpleServo clawRightServo;
    SimpleServo clawLeftServo;
    private double maxAngle, minAngle;
    private final double maxPosition = 1;
    private final double minPosition = 0;
    @Override
    public void runOpMode() throws InterruptedException {

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
    }
}
