package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "TouchSensorTest", group = "Sensor")
public class TouchSensorTest extends OpMode
{
    private TouchSensor touchSensor;
    private boolean isTouching = false;
    @Override
    public void init()
    {
        touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");

        if (touchSensor != null) {
            telemetry.addData("Touch Sensor", "Initialized");
        } else {
            telemetry.addData("Touch Sensor", "Not Found");
        }
        telemetry.update();

    }

    @Override
    public void loop()
    {
        //isTouching = touchSensor.isPressed();

        if (touchSensor != null)
        {
            telemetry.addData("Touch Sensor Pressed:", touchSensor.isPressed());
            telemetry.addData("Touch Sensor Force:", touchSensor.getValue());
        } else
        {
            telemetry.addData("Touch Sensor", "Not Found");
        }
        telemetry.update();
    }

}

