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
    public static final double WHEEL_DIAMETER = 9.6;
    public static final double PULSES = 537.7;
    public static final double PI = 3.1415;
    public static final double PULSES_PER_CM = PULSES / (WHEEL_DIAMETER * PI);
    public static final double verclaw_open = 0.4;
    public static final double verclaw_close = 0.63;
    public static final double verrotate_chamber = 0.24;
    public static final double verrotate_player = 0.89;
    public static final double horclaw_open = 0.3694;
    public static final double horclaw_close = 0.6;
    public static final double horrotate_ground = 1;

    public static final double horrotate_middle = 0.47;
    public static final double horrotate_transfer = 0.3294;
    public static final int middle_chamber = -1300;
    public static final int high_chamber = -3000;
    public static final int high_basket = -4200;
    public static final int sample = -1700;
    public static final int transfer = 1500;
    public static final int low_basket = -1000;
    public static final int way_basket = 19;
    public static final int way_sample_1 = 7;
    public static final int way_sample_2 = 30;




}