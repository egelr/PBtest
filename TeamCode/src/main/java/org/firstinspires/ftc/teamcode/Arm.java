package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

@TeleOp(name = "Encoder Test ")
public class Arm extends OpMode {
    DcMotor armLeftMotor;
    double ticksPerRevolution = 537.6; // Update this value with the correct value for your specific GoBilda motor
    double newTarget;

    @Override
    public void init() {
        armLeftMotor = hardwareMap.get(DcMotor.class, "motor");
        telemetry.addData("Hardware: ", "Initialized");

        // Set motor mode and reset encoder
        armLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set motor type for GoBilda 5202 series motor
        armLeftMotor.setMotorType(MotorConfigurationType.getMotorType(MotorConfigurationType.MOTOR_TYPE_GOBILDA_5203_100));
    }

    @Override
    public void loop() {
        if (gamepad1.square) {
            encoder(2);
        }

        telemetry.addData("Motor Ticks: ", armLeftMotor.getCurrentPosition());

        if (gamepad1.triangle) {
            tracker();
        }
    }

    public void encoder(int turnage) {
        newTarget = ticksPerRevolution / turnage;
        armLeftMotor.setTargetPosition((int) newTarget);
        armLeftMotor.setPower(0.3);
        armLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void tracker() {
        armLeftMotor.setTargetPosition(0);
        armLeftMotor.setPower(0.8);
        armLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
