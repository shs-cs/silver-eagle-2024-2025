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

    public double TwistyTurnySidePosition = 0.185;
    public double TwistyTurnyStraight = 0.53;
    public double TwistyTurnyFlipPosition = 0.185;
    public double ClawOpenNormalPos =  0.2;
    public double ClawOpenWidePos = 0.1;
    public double ClawNomNom = 0.31;

    public double WristRestPosition = 0.9;
    public double WristGrabbingPosition = 0;
    public double WristSlammaJammaPosition = 0.17;

    public double WristSpecimenPosition = 0.37;

    public double WristBackScoringPosition = 0.097;
    public double WristHighBasketPosition = 0.33;


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

        @Override
        public boolean run(@NonNull TelemetryPacket packet)
        {
                LeftGripperServo.setPosition(ClawOpenWidePos);
                RightGripperServo.setPosition(ClawOpenWidePos);
                return false; // call run again
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

    public class HomePosition implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            WristServo.setPosition(WristRestPosition);
            TwistyTurnyServo.setPosition(TwistyTurnyStraight);
            LeftGripperServo.setPosition(ClawNomNom);
            RightGripperServo.setPosition(ClawNomNom);
            return false;
        }

    }

    public Action homePosition() {
        return new HomePosition();
    }



    public class WristSpecimen implements Action {

        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet)
        {
            if (!initialized) {
                WristServo.setPosition(WristSpecimenPosition);

                initialized = true;
                return true; // call run again
            } else {
                if (WristServo.getPosition() == WristSpecimenPosition) {
                    return false; // stop call run
                }
                return true; // call this function again
            }


        }

    }

    public Action wristSpecimen()
    {
        return new WristSpecimen();
    }







    // TODO:
    // - move claw to specimen position
    // move claw to sample psoition
    // pick up sample position

    // go back inside position

}


