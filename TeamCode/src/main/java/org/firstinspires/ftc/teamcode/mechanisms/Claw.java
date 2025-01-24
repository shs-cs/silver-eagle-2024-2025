package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
public class Claw {
    private Servo LeftGripperServo;
    private Servo RightGripperServo;
    private Servo WristServo;
    private Servo TwistyTurnyServo;

    public double TwistyTurnySidePosition = 0.39;
    public double TwistyTurnyStraight = 0.75;
    public double TwistyTurnyFlipPosition = 0.08; //0.675
    public double ClawOpenNormalPos =  0.2;
    public double ClawOpenWidePos = 0.1;
    public double ClawNomNom = 0.31;

    public double WristRestPosition = 0.16;
    public double WristGrabbingPosition = 0.016;
    public double WristSlammaJammaPosition = 0.17;

    public double WristSpecimenPosition = 0.064;

    public double WristBackScoringPosition = 0.097;
    public double WristHighBasketPosition = 0.064;

    public Claw(HardwareMap hardwareMap) {
        TwistyTurnyServo = hardwareMap.get(Servo.class, "TwistyTurnyServo");
        LeftGripperServo = hardwareMap.get(Servo.class, "LeftGripperServo");
        RightGripperServo = hardwareMap.get(Servo.class, "RightGripperServo");
        WristServo = hardwareMap.get(Servo.class, "WristServo");

        // Reverse any servos that need to be reversed
        RightGripperServo.setDirection(Servo.Direction.REVERSE);
        WristServo.setDirection(Servo.Direction.REVERSE);
    }
    public class ClawOpen implements Action
    {

        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet)
        {
            if (!initialized) {
                LeftGripperServo.setPosition(ClawOpenNormalPos);
                RightGripperServo.setPosition(ClawOpenNormalPos);
                initialized = true;
                return true; // call run again
            } else {
                if (LeftGripperServo.getPosition() == ClawOpenNormalPos && RightGripperServo.getPosition() == ClawOpenNormalPos) {
                    return false; // stop call run
                }
                return true; // call this function again
            }


        }

    }
    public Action openClaw()
    {
        return new ClawOpen();
    }


    public class ClawClose implements Action
    {

        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet)
        {
            if (!initialized) {
                LeftGripperServo.setPosition(ClawNomNom);
                RightGripperServo.setPosition(ClawNomNom);
                initialized = true;
                return true; // call run again
            } else {
                if (LeftGripperServo.getPosition() == ClawNomNom && RightGripperServo.getPosition() == ClawNomNom) {
                    return false; // stop call run
                }
                return true; // call this function again
            }


        }

    }
    public Action clawClose()
    {
        return new ClawClose();
    }



    public class WristGrab implements Action {

        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet)
        {
            if (!initialized) {
                WristServo.setPosition(WristGrabbingPosition);

                initialized = true;
                return true; // call run again
            } else {
                if (WristServo.getPosition() == WristGrabbingPosition) {
                    return false; // stop call run
                }
                return true; // call this function again
            }


        }

    }

    public Action wristGrab()
    {
        return new WristGrab();
    }

    public class WristHighBasket implements Action {

        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet)
        {
            if (!initialized) {
                WristServo.setPosition(WristHighBasketPosition);

                initialized = true;
                return true; // call run again
            } else {
                if (WristServo.getPosition() == WristHighBasketPosition) {
                    return false; // stop call run
                }
                return true; // call this function again
            }


        }

    }

    public Action wristHighBasket()
    {
        return new WristHighBasket();
    }


    public class WristBack implements Action {

        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet)
        {
            if (!initialized) {
                WristServo.setPosition(WristRestPosition);

                initialized = true;
                return true; // call run again
            } else {
                if (WristServo.getPosition() == WristRestPosition) {
                    return false; // stop call run
                }
                return true; // call this function again
            }


        }

    }

    public Action wristBack()
    {
        return new WristBack();
    }






    // TODO:
    // - move claw to specimen position
    // move claw to sample psoition
    // pick up sample position

    // go back inside position

}


