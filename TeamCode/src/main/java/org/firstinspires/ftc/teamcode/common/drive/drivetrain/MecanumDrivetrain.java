package org.firstinspires.ftc.teamcode.common.drive.drivetrain;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.drivebase.RobotDrive;
import com.qualcomm.robotcore.hardware.DcMotorEx;
//import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import org.firstinspires.ftc.teamcode.common.util.wrappers.WSubsystem;
//import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Pose;
//import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.Vector2D;

import java.util.Arrays;
public class MecanumDrivetrain extends LinearOpMode {
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
