package org.firstinspires.ftc.teamcode.AUTO;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Globals {
    // DC Motors
    public static DcMotor LeftFront = null;
    public static DcMotor LeftBack = null;
    public static DcMotor RightFront = null;
    public static DcMotor RightBack = null;
    public static DcMotor Vertical = null;
    public static DcMotor Horizontal = null;

    // IMU
    public static IMU imu;

    // Servos
    public static Servo VerRotate;
    public static Servo VerClaw;
    public static Servo HorRotate;
    public static Servo HorClaw;

    // Sensors
    public static TouchSensor touchVertical;
    public static TouchSensor touchHorizontal;

    // Constant
    public static final double WHEEL_DIAMETER = 10.4;
    public static final double PULSES = 537.7;
    public static final double PI = 3.1415;
    public static final double PULSES_PER_CM = PULSES / (WHEEL_DIAMETER * PI);
}
