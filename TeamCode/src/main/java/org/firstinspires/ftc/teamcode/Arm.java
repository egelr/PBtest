package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.robot.Robot;

@TeleOp(name = "Encoder Test ")
public class Arm extends LinearOpMode{
    private DcMotor armLiftMotor;
    static  final double MOTOR_TICK_COUNT = 1120;
    public void runOpMode() throws InterruptedException{
        double quarterTurn = 1120/4;
        //new Motor(hardwareMap, "armLiftMotot", Motor.GoBILDA.RPM_60);

        //telemetry.addData("Status", "Resetting Encoders:");
        //telemetry.update();

        //waitForStart();

        //Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armLiftMotor = hardwareMap.get(DcMotor.class, "motor");

        telemetry.addData("Hardware: ", "Initialized");
        telemetry.update();

        waitForStart();

        // Set motor mode and reset encoder
        armLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int newTarget = armLiftMotor.getTargetPosition() + (int)quarterTurn;
        armLiftMotor.setTargetPosition(newTarget);
        armLiftMotor.setPower(1);
        armLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (armLiftMotor.isBusy()){
            telemetry.addData("Status ", "Running the motor to a quarter turn");
            telemetry.update();
        }
        armLiftMotor.setPower(0);
        armLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        newTarget = armLiftMotor.getTargetPosition() - (int)quarterTurn;
        armLiftMotor.setTargetPosition(newTarget);
        armLiftMotor.setPower(1);
        armLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Path", "Complete");
        telemetry.update();

        if (gamepad1.triangle){
            armLiftMotor.setPower(1);
        }
        telemetry.addData("Motor Ticks: ", armLiftMotor.getCurrentPosition());
        if (gamepad1.square){
            armLiftMotor.setPower(0);
        }
    }
}
