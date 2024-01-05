package org.firstinspires.ftc.teamcode.common.util.wrappers;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.hardware.motors.Motor;

public class WEncoder {
    Motor m_motor;
    Motor m_motor_2;
    double sp;

    public void getM_motor() {

        m_motor.setRunMode(Motor.RunMode.PositionControl);
        m_motor.setPositionCoefficient(0.05);
        double kP = m_motor.getPositionCoefficient();
        int pos = m_motor.getCurrentPosition();
        m_motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        m_motor.setPositionTolerance(30);
        telemetry.addData("Status m2", m_motor.getCurrentPosition());
        telemetry.update();
        //return null;
    }

    //m_motor_2.setInverted(true);

    public void getM_motor_2() {
        m_motor_2.setRunMode(Motor.RunMode.PositionControl);
        m_motor_2.setPositionCoefficient(0.05);
        double kP_2 = m_motor_2.getPositionCoefficient();
        m_motor_2.setInverted(true);
        m_motor_2.resetEncoder();
        m_motor_2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        m_motor_2.set(0);
        m_motor_2.setPositionTolerance(5);
        telemetry.addData("Status m2", m_motor_2.getCurrentPosition());
        telemetry.update();
        //return null;
    }


    //GamepadEx myGamepad = new Gamepad(gamepad1);


    public void setTargetPosition(int i) {
        //m_motor.setTargetPosition(1540);
    }

    public boolean atTargetPosition() {
        return false;
    }

    public Object getCurrentPosition() {
        return null;
    }

    public void stopMotor() {
    }

    public void setPower(double v) {
    }

    public void set(double sp) {
        sp = 0.02;
    }
}
