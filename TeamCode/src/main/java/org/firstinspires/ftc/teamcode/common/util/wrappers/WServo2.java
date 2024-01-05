package org.firstinspires.ftc.teamcode.common.util.wrappers;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.SimpleServo;

public class WServo2 extends SimpleServo {
    SimpleServo clawAngleServo1;
    SimpleServo clawRightServo1;
    SimpleServo clawLeftServo1;

    public WServo2(HardwareMap hw, String servoName, double minAngle, double maxAngle, AngleUnit angleUnit) {
        super(hw, servoName, minAngle, maxAngle, angleUnit);
    }

    public void turnToAngle(int i) {
        clawAngleServo1 = new SimpleServo(
                hardwareMap, "ClawAngleServo", 10, 120,
                AngleUnit.DEGREES
        );

        clawRightServo1 = new SimpleServo(
                hardwareMap, "ClawRightServo", 10, 120,
                AngleUnit.DEGREES
        );

        clawLeftServo1 =  new SimpleServo(
                hardwareMap, "ClawLeftServo", 10, 120,
                AngleUnit.DEGREES
        );
        //waitForStart();


        clawAngleServo1.setInverted(true);
    }
}
