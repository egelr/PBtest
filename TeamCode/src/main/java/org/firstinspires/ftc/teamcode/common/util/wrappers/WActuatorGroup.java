/*package org.firstinspires.ftc.teamcode.common.util.wrappers;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.profile.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.profile.ProfileConstraints;
import org.firstinspires.ftc.teamcode.common.drive.pathing.geometry.profile.ProfileState;
import org.firstinspires.ftc.teamcode.common.hardware.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.common.util.MathUtils;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.DoubleSupplier;

public class WActuatorGroup {
    public enum FeedforwardMode {
        NONE,
        CONSTANT,
        ANGLE_BASED,
        ANGLE_BASED_SIN
    }

    private final Map<String, HardwareDevice> devices = new HashMap<>();
    private AsymmetricMotionProfile profile;
    private ProfileConstraints constraints;
    private ProfileState state;
    private PIDController controller;
    private DoubleSupplier voltage;
    public ElapsedTime timer;
    private RobotHardware robot = RobotHardware.getInstance();

    private double position = 0.0;
    private double targetPosition = 0.0;
    private double power = 0.0;
    private double tolerance = 0.0;
    private double feedforwardMin = 0.0;
    private double feedforwardMax = 0.0;
    private double currentFeedforward = 0.0;
    private double targetPositionOffset = 0.0;
    private double offset = 0.0;

    private boolean reached = false;

    private FeedforwardMode mode = FeedforwardMode.NONE;

    /**
     * Actuator constructor with varargs HardwareDevice parameter
     *
     * @param devices

    public WActuatorGroup(HardwareDevice... devices) {

}
*/