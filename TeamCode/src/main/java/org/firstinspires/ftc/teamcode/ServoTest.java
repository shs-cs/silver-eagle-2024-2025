package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "ServoTest", group = "TeleOp")
public class ServoTest extends OpMode
{
    public static class Params {
        public double rightGripperPosition = 0.0;
        public double leftGripperPosition = 0.0;
        public double wristPosition = 0.0;
        public double twistyPosition = 0.0;
    }
    public static Params PARAMS = new Params();

    private Servo twistyTurnyServo;
    private Servo rightGripperServo;
    private Servo leftGripperServo;
    private Servo wristServo;



    @Override
    public void init() {
        twistyTurnyServo = hardwareMap.get(Servo.class, "TwistyTurnyServo");
        wristServo = hardwareMap.get(Servo.class, "WristServo");
        leftGripperServo = hardwareMap.get(Servo.class, "LeftGripperServo");
        rightGripperServo = hardwareMap.get(Servo.class, "RightGripperServo");
        rightGripperServo.setDirection(Servo.Direction.REVERSE);
        wristServo.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            twistyTurnyServo.setPosition(PARAMS.twistyPosition);
        }
        if (gamepad1.b) {
            wristServo.setPosition(PARAMS.wristPosition);
        }
        if (gamepad1.x) {
            leftGripperServo.setPosition(PARAMS.leftGripperPosition);
        }
        if (gamepad1.y) {
            rightGripperServo.setPosition(PARAMS.rightGripperPosition);
        }

        if (gamepad1.dpad_up) {
            wristServo.setDirection(Servo.Direction.REVERSE);
            wristServo.setPosition(PARAMS.twistyPosition);
            wristServo.setDirection(Servo.Direction.FORWARD);
        }



    }
}
