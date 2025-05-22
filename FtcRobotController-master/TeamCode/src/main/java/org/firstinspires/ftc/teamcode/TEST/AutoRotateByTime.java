package org.firstinspires.ftc.teamcode.TEST;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Auto Rotate By Time", group="Robot")
@Disabled
public class AutoRotateByTime extends LinearOpMode {
    private DcMotor LeftFront = null;
    private DcMotor LeftBack = null;
    private DcMotor RightFront = null;
    private DcMotor RightBack = null;

    static final double ROTATE = 0.3;

    @Override
    public void runOpMode() {
        LeftFront = hardwareMap.get(DcMotor.class, "left_front");
        LeftBack = hardwareMap.get(DcMotor.class, "left_back");
        RightFront = hardwareMap.get(DcMotor.class, "right_front");
        RightBack = hardwareMap.get(DcMotor.class, "right_back");

        LeftFront.setDirection(DcMotor.Direction.REVERSE);
        LeftBack.setDirection(DcMotor.Direction.REVERSE);
        RightFront.setDirection(DcMotor.Direction.FORWARD);
        RightBack.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        LeftFront.setPower(ROTATE);
        LeftBack.setPower(ROTATE);
        RightFront.setPower(-ROTATE);
        RightBack.setPower(-ROTATE);
        sleep(2000);

        LeftFront.setPower(-ROTATE);
        LeftBack.setPower(-ROTATE);
        RightFront.setPower(ROTATE);
        RightBack.setPower(ROTATE);
        sleep(2000);
    }
}