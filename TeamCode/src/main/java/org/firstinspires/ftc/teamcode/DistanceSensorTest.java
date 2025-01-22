package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Distance Sensor Test", group = "TeleOp")
public class DistanceSensorTest extends OpMode
{
    private DistanceSensor distanceSensor;

    @Override
    public void init()
    {
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        if (distanceSensor != null)
        {
            telemetry.addData("distance Sensor", "Initialized");
        } else
        {
            telemetry.addData("distance Sensor", "Not Found");
        }
        telemetry.update();

    }

    @Override
    public void loop()
    {
        //double distanceCm = distanceSensor.getDistance(DistanceUnit.CM);
        double distanceInch = distanceSensor.getDistance(DistanceUnit.INCH);

        //telemetry.addData("Distance (cm):", distanceCm);
        telemetry.addData("Distance (inches):", distanceInch);
        telemetry.update();
    }






}


