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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp
public class test_opmode extends LinearOpMode {
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


        GamepadEx driverOp = new GamepadEx(gamepad1);


        waitForStart();

        while (!isStopRequested()) {

            drive.driveRobotCentric(
                    driverOp.getLeftX(),
                    driverOp.getLeftY(),
                    driverOp.getRightX(),
                    false

            );

        }



    }


}

