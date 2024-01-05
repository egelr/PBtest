package org.firstinspires.ftc.teamcode;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;


@TeleOp(name = "Encoder Test ")
public class Arm extends LinearOpMode {
    private DcMotor armLiftMotor;
    private DcMotor armSlideMotor;
    static final double MOTOR_TICK_COUNT = 8192;

    public void runOpMode() throws InterruptedException {

        int quarterTurn = 8192;

        Motor m_motor = new Motor(hardwareMap, "armLiftMotor", 8192, 60);
        Motor m_motor_2 = new Motor(hardwareMap, "armSlideMotor", 1425.2, 117);
        telemetry.addData("Hardware: ", "Initialized");
        telemetry.update();
        m_motor.setRunMode(Motor.RunMode.PositionControl);
        m_motor_2.setRunMode(Motor.RunMode.PositionControl);
        //m_motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

// set and get the position coefficient
        m_motor.setPositionCoefficient(0.05);
        double kP = m_motor.getPositionCoefficient();
        m_motor_2.setPositionCoefficient(0.05);
        double kP_2 = m_motor_2.getPositionCoefficient();


        //m_motor_2.setInverted(true);
        int pos = m_motor.getCurrentPosition();
        //m_motor.setTargetPosition(pos);
        m_motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        m_motor_2.setInverted(true);
        //int pos_2 = m_motor_2.getCurrentPosition();
        m_motor_2.resetEncoder();
        //m_motor_2.getInverted(false);
        //int targetPos_2 = pos_2;
        //m_motor_2.setTargetPosition(pos_2);
        m_motor_2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        m_motor_2.set(0);
// set the tolerance
        m_motor.setPositionTolerance(30);
        m_motor_2.setPositionTolerance(5);// allowed maximum error
        //GamepadEx myGamepad = new Gamepad(gamepad1);
        double sp = 0.02;
        telemetry.addData("Status m2", m_motor_2.getCurrentPosition());
        telemetry.update();

        waitForStart();


// perform the control loop
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

}





