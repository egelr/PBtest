/*package org.firstinspires.ftc.teamcode.common.hardware;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;

//import org.apache.commons.math3.analysis.function.Inverse;
import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.profile.ProfileConstraints;
//import org.firstinspires.ftc.teamcode.common.util.InverseKinematics;
//import org.firstinspires.ftc.teamcode.common.util.wrappers.WActuatorGroup;
//import org.firstinspires.ftc.teamcode.common.util.wrappers.WEncoder;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WServo2;
//import org.firstinspires.ftc.teamcode.common.util.wrappers.WSubsystem;
//import org.firstinspires.ftc.teamcode.common.util.wrappers.WServo;

//import java.util.ArrayList;
//import java.util.Arrays;
//import java.util.List;


public class RobotHardware {
    private static RobotHardware instance = null;
    public boolean enabled;
    private ElapsedTime voltageTimer = new ElapsedTime();
    private double voltage = 12.0;
    public DcMotorEx dtFrontLeftMotor;
    public DcMotorEx dtFrontRightMotor;
    public DcMotorEx dtBackLeftMotor;
    public DcMotorEx dtBackRightMotor;
    public MecanumDrive drive;
    private HardwareMap hardwareMap;
    //public WServo clawAngleServo;
    //public WServo clawLeftServo;
    //public WServo clawRightServo;
    //private ArrayList<WSubsystem> subsystems;
    //public List<LynxModule> modules;
    public WServo2 clawAngleServo1;
    public WServo2 clawLeftServo1;
    public WServo2 clawRightServo1;

    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        instance.enabled = true;
        return instance;
    }

    public void init(final HardwareMap hardwareMap, final Telemetry telemetry) {
        //modules = hardwareMap.getAll(LynxModule.class);
        //modules.get(0).setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        //modules.get(1).setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        this.hardwareMap = hardwareMap;
        // DRIVETRAIN
        /*
        this.dtBackLeftMotor = hardwareMap.get(DcMotorEx.class, "dtBackLeftMotor");
        dtBackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.dtFrontLeftMotor = hardwareMap.get(DcMotorEx.class, "dtFrontLeftMotor");
        dtFrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.dtBackRightMotor = hardwareMap.get(DcMotorEx.class, "dtBackRightMotor");
        dtBackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dtBackRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.dtFrontRightMotor = hardwareMap.get(DcMotorEx.class, "dtFrontRightMotor");
        dtFrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dtFrontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);



        MecanumDrive drive = new MecanumDrive(
                new Motor(hardwareMap, "dtFrontLeftMotor", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "dtFrontRightMotor", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "dtBackLeftMotor", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap, "dtBackRightMotor", Motor.GoBILDA.RPM_312)
        );


        //INTAKE
        Motor m_motor = new Motor(hardwareMap, "armLiftMotor", 8192, 60);
        Motor m_motor_2 = new Motor(hardwareMap, "armSlideMotor", 1425.2, 117);
        clawAngleServo1 = new WServo2(hardwareMap.get(Servo.class, "clawAngleServo"));
        clawLeftServo1 = new WServo2(hardwareMap.get(Servo.class, "ClawLeftServo"));
        clawRightServo1 = new WServo2(hardwareMap.get(Servo.class, "ClawRightServo"));
        clawRightServo1.setDirection(Servo.Direction.REVERSE);

        //this.intakePivotActuator = new WActuatorGroup(clawAngleServo);
    }
/*
    public void addSubsystem(MecanumDrivetrain drivetrain, ClawSubsystem claw) {
        //added
    }
    /*
    public void read() {
//        imuAngle = imu.getAngularOrientation().firstAngle;
        for (WSubsystem subsystem : subsystems) {
            subsystem.read();
        }
    }

    public void write() {
        for (WSubsystem subsystem : subsystems) {
            subsystem.write();
        }
    }

    public void periodic() {
        if (voltageTimer.seconds() > 5) {
            voltageTimer.reset();
            voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        }

        for (WSubsystem subsystem : subsystems) {
            subsystem.periodic();
        }
    }
    public void reset() {
        for (WSubsystem subsystem : subsystems) {
            subsystem.reset();
        }
    }

    public void clearBulkCache() {
        modules.get(0).clearBulkCache();
        modules.get(1).clearBulkCache();
    }
}
     */
