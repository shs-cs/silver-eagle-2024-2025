package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "ColorSensorTest", group = "TeleOp")
public class ColorSensorTest extends OpMode {
    private ColorSensor colorSensor;
    private boolean isBlue = false;
    private boolean isRed = false;
    private boolean isYellow = false;

    @Override
    public void init()
    {

        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        if (colorSensor != null) {
            telemetry.addData("color Sensor", "Initialized");
        } else {
            telemetry.addData("color Sensor", "Not Found");
        }
        telemetry.update();


    }

    @Override
    public void loop()
    {

        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();


        isBlue = blue > red && blue > green && blue > 100;
        isRed = red > blue && red > green && red > 100;
        isYellow = red > blue && green > blue && red > 100 && green > 100;




        telemetry.addData("Red Value:", red);
        telemetry.addData("Green Value:", green);
        telemetry.addData("Blue Value:", blue);
        telemetry.addData("Is Blue:", isBlue);
        telemetry.addData("Is Red:", isRed);
        telemetry.addData("Is Yellow:", isYellow);
        telemetry.update();
    }
}
