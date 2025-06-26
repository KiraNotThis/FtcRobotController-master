package org.firstinspires.ftc.teamcode.AUTO;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Distance Sensor Telemetry", group = "Autonomous")
public class DistanceSensorTelemetry extends LinearOpMode {

    private DistanceSensor distanceSensor;

    @Override
    public void runOpMode() {

        // Инициализация датчика
        distanceSensor = hardwareMap.get(DistanceSensor.class, "sensor_distance");

        telemetry.addData("Статус", "Готов к запуску");
        telemetry.update();

        waitForStart();

        // Основной цикл после запуска
        while (opModeIsActive()) {
            double distance = distanceSensor.getDistance(DistanceUnit.CM);

            telemetry.addData("Distance (cm)", "%.2f", distance);
            telemetry.update();

            sleep(1000); // Пауза 1 секунда между измерениями
        }
    }
}
