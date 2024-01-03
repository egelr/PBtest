package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.arcrobotics.ftclib.hardware.motors.Motor;

@TeleOp(name = "Encoder Test ")
public class Arm extends LinearOpMode {
    private DcMotor armLiftMotor;
    static final double MOTOR_TICK_COUNT = 8192;

    public void runOpMode() throws InterruptedException {

        int quarterTurn = 8192;

        Motor m_motor = new Motor(hardwareMap, "armLiftMotor", 8192, 60);

        telemetry.addData("Hardware: ", "Initialized");
        telemetry.update();
        m_motor.setRunMode(Motor.RunMode.PositionControl);
        //m_motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

// set and get the position coefficient
        m_motor.setPositionCoefficient(0.05);
        double kP = m_motor.getPositionCoefficient();

// set the target position
        m_motor.encoder.reset();
        m_motor.setInverted(false);
        m_motor.setTargetPosition(-500);      // an integer representing
        // desired tick count

        m_motor.set(0);

// set the tolerance
        m_motor.setPositionTolerance(13.6);   // allowed maximum error

        waitForStart();

        // Set motor mode and reset encoder
        //armLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //armLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    //    armLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //armLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //int newTarget = armLiftMotor.getTargetPosition() + 1000;
//        int newTarget = 500;
        //armLiftMotor.setTargetPosition(newTarget);
        //armLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //armLiftMotor.setPower(0.5);

        // set the run mode

// perform the control loop
        while (!m_motor.atTargetPosition()) {
            m_motor.set(0.1);
            telemetry.addData("Status ", m_motor.getCurrentPosition());
            telemetry.update();

        }
        m_motor.stopMotor(); // stop the motor

        //while (opModeIsActive() && armLiftMotor.isBusy()) {
        //    telemetry.addData("Status ", armLiftMotor.getCurrentPosition());
         //   telemetry.update();
       // }
        //armLiftMotor.setPower(0);

        telemetry.addData("Path", "Complete");
        telemetry.update();

    }
}

