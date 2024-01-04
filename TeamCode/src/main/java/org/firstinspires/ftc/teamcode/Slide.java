package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.arcrobotics.ftclib.hardware.motors.Motor;
/*
@TeleOp(name = "Encoder Test Nr2 ")
public class Slide extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        Motor m_motor_2 = new Motor(hardwareMap, "armSlideMotor", 1425.2, 117);
        telemetry.addData("Hardware: ", "Initialized");
        telemetry.update();
        m_motor_2.setRunMode(Motor.RunMode.PositionControl);

        m_motor_2.setPositionCoefficient(0.05);
        double kP_2 = m_motor_2.getPositionCoefficient();

        m_motor_2.setInverted(false);
        int pos_2 = m_motor_2.getCurrentPosition();
        int targetPos_2 = pos_2;
        m_motor_2.setTargetPosition(pos_2);
        m_motor_2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        m_motor_2.setPositionTolerance(30);// allowed maximum error
        //GamepadEx myGamepad = new Gamepad(gamepad1);
        double sp = 0.05;
        waitForStart();

        while (opModeIsActive()){
            if (gamepad1.square) {
                sp = 0.01;
                m_motor_2.setTargetPosition(500);
                telemetry.addData("share", "lllll");
                telemetry.update();
            }

            if (gamepad1.triangle) {
                sp = 0.01;
                m_motor_2.setTargetPosition(-500);
                telemetry.addData("options", "llll");
                telemetry.update();

            }


            if (!m_motor_2.atTargetPosition()) {
                m_motor_2.set(sp);
                telemetry.addData("Status", "Moving to target position");
                telemetry.addData("Current Position", m_motor_2.getCurrentPosition());
                telemetry.addData("Target Position", targetPos_2);
                telemetry.update();
            }
            else {
                m_motor_2.stopMotor();
                telemetry.addData("Status", "At target position");
                telemetry.addData("Current Position", m_motor_2.getCurrentPosition());
                telemetry.addData("Target Position", targetPos_2);
                telemetry.update();
                sleep(10); // Sleep to avoid busy-waiting
            }
            m_motor_2.stopMotor();
            sleep(10);
        }
    }
}
*/