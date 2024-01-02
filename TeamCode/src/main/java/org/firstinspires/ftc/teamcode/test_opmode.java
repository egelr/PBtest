package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;


@TeleOp
public class test_opmode extends LinearOpMode {

    // This variable determines whether the following program
    // uses field-centric or robot-centric driving styles. The
    // differences between them can be read here in the docs:
    // https://docs.ftclib.org/ftclib/features/drivebases#control-scheme
    static final boolean FIELD_CENTRIC = false;


    @Override
    public void runOpMode() throws InterruptedException {
        // constructor takes in frontLeft, frontRight, backLeft, backRight motors
        // IN THAT ORDER
        MecanumDrive drive = new MecanumDrive(
                new Motor(hardwareMap, "dtFrontLeftMotor", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "dtFrontRightMotor", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "dtBackLeftMotor", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "dtBackRightMotor", Motor.GoBILDA.RPM_312)
        );

        // This is the built-in IMU in the REV hub.
        // We're initializing it by its default parameters
        // and name in the config ('imu'). The orientation
        // of the hub is important. Below is a model
        // of the REV Hub and the orientation axes for the IMU.
        //
        //                           | Z axis
        //                           |
        //     (Motor Port Side)     |   / X axis
        //                       ____|__/____
        //          Y axis     / *   | /    /|   (IO Side)
        //          _________ /______|/    //      I2C
        //                   /___________ //     Digital
        //                  |____________|/      Analog
        //
        //                 (Servo Port Side)
        //
        // (unapologetically stolen from the road-runner-quickstart)

        //RevIMU imu = new RevIMU(hardwareMap);
        //imu.init();

        // the extended gamepad object
        GamepadEx driverOp = new GamepadEx(gamepad1);

        waitForStart();

        while (!isStopRequested()) {

            // Driving the mecanum base takes 3 joystick parameters: leftX, leftY, rightX.
            // These are related to the left stick x value, left stick y value, and
            // right stick x value respectively. These values are passed in to represent the
            // strafing speed, the forward speed, and the turning speed of the robot frame
            // respectively from [-1, 1].

            drive.driveRobotCentric(
                    driverOp.getLeftX(),
                    driverOp.getLeftY(),
                    driverOp.getRightX(),
                    false

            );

        }
    }

@TeleOp(name = "Encoder Test ")
public class Encoder extends OpMode {
    DcMotor armLiftMotor;
    double ticksPerRevolution = 537.6; // Update this value with the correct value for your specific GoBilda motor
    double newTarget;

    @Override
    public void init() {
        armLiftMotor = hardwareMap.get(DcMotor.class, "motor");
        telemetry.addData("Hardware: ", "Initialized");

        // Set motor mode and reset encoder
        armLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set motor type for GoBilda 5202 series motor
        //armLiftMotor.setMotorType(MotorConfigurationType.getMotorType(MotorConfigurationType.Motor.GoBILDA.RPM_60));
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            encoder(2);
        }

        telemetry.addData("Motor Ticks: ", armLiftMotor.getCurrentPosition());

        if (gamepad1.b) {
            tracker();
        }
    }

    public void encoder(int turnage) {
        newTarget = ticksPerRevolution / turnage;
        armLiftMotor.setTargetPosition((int) newTarget);
        armLiftMotor.setPower(0.3);
        armLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void tracker() {
        armLiftMotor.setTargetPosition(0);
        armLiftMotor.setPower(0.8);
        armLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}

}

