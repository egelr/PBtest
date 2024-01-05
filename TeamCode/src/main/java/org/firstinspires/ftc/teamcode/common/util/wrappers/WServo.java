package org.firstinspires.ftc.teamcode.common.util.wrappers;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.SimpleServo;

public class WServo implements Servo {

    private Servo clawAngleServo;
    private Servo clawRightServo;
    private Servo clawLeftServo;
    private double offset = 0.0;

    public WServo(Servo servo) {
        this.clawAngleServo = servo;
    }

    public void setOffset(double offset) {
        this.offset = offset;
    }

    public double getOffset() {
        return this.offset;
    }

    @Override
    public Manufacturer getManufacturer() {
        return null;
    }

    @Override
    public String getDeviceName() {
        return clawAngleServo.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return null;
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void close() {

    }

    @Override
    public ServoController getController() {
        return clawAngleServo.getController();
    }

    @Override
    public int getPortNumber() {
        return clawAngleServo.getPortNumber();
    }

    @Override
    public void setDirection(Direction direction) {
        this.clawAngleServo.setDirection(direction);
    }

    @Override
    public Direction getDirection() {
        return this.clawAngleServo.getDirection();
    }

    @Override
    public void setPosition(double position) {
        this.clawAngleServo.setPosition(position - offset);
    }

    @Override
    public double getPosition() {
        return this.clawAngleServo.getPosition();
    }

    @Override
    public void scaleRange(double min, double max) {
        this.clawAngleServo.scaleRange(min, max);
    }




}

