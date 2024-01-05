package org.firstinspires.ftc.teamcode.common.hardware;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.apache.commons.math3.analysis.function.Inverse;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.drive.drivetrain.MecanumDrivetrain;
//import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.profile.ProfileConstraints;
import org.firstinspires.ftc.teamcode.common.subsystem.ClawSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystem.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.common.util.InverseKinematics;
//import org.firstinspires.ftc.teamcode.common.util.wrappers.WActuatorGroup;
//import org.firstinspires.ftc.teamcode.common.util.wrappers.WEncoder;
//import org.firstinspires.ftc.teamcode.common.util.wrappers.WSubsystem;
import org.firstinspires.ftc.teamcode.common.util.wrappers.WServo;
public class RobotHardware {
    private static RobotHardware instance = null;
    public boolean enabled;
    public DcMotorEx dtFrontLeftMotor;
    public DcMotorEx dtFrontRightMotor;
    public DcMotorEx dtBackLeftMotor;
    public DcMotorEx dtBackRightMotor;
    private HardwareMap hardwareMap;
    public WServo clawAngleServo;
    public WServo clawLeftServo;
    public WServo clawRightServo;

    public WServo intakePivotLeftServo;
    public WServo intakePivotRightServo;

    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        instance.enabled = true;
        return instance;
    }

    public void init(final HardwareMap hardwareMap, final Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        // DRIVETRAIN
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

        //CLAW
        clawAngleServo = new WServo(hardwareMap.get(Servo.class, "clawAngleServo"));
        clawLeftServo = new WServo(hardwareMap.get(Servo.class, "ClawleftServo"));
        clawRightServo = new WServo(hardwareMap.get(Servo.class, "ClawRightServo"));
        clawRightServo.setDirection(Servo.Direction.REVERSE);
    }

    public void addSubsystem(MecanumDrivetrain drivetrain, ClawSubsystem claw, IntakeSubsystem intake) {
        //added
    }
}
