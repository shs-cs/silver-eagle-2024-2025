package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SlideySlide {
    private DcMotorEx ViperMotor;

    public SlideySlide(HardwareMap hardwareMap) {
        ViperMotor = hardwareMap.get(DcMotorEx.class, "ViperMotor");
        ViperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ViperMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        ViperMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    private static final int highBarPosition = -40;

    private static final int specimenPos = -20;

    private static final int highBarPos = -30;

    private static final int resetPos = 0;
    public class SlideHighBasket implements Action
    {
        private boolean initialized = false;

        // actions are formatted via telemetry packets as below
        // @return true means function should rerun
        // @return false means action is complete
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {
                ViperMotor.setPower(0.8);
                initialized = true;
            }

            if (ViperMotor.getCurrentPosition() <= highBarPosition) {
                ViperMotor.setPower(0);
                return false;
            } else {
                if (!ViperMotor.isBusy()) {
                    ViperMotor.setTargetPosition(highBarPosition);
                    ViperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                return true;
            }
        }
    }
    public Action slideHighBasket()
    {
        return new SlideHighBasket();
    }

    public class SlideSpecimen implements Action
    {
        private boolean initialized = false;

        // actions are formatted via telemetry packets as below
        // @return true means function should rerun
        // @return false means action is complete
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {
                ViperMotor.setPower(0.8);
                initialized = true;
            }

            if (ViperMotor.getCurrentPosition() <= specimenPos) {
                ViperMotor.setPower(0);
                return false;
            } else {
                if (!ViperMotor.isBusy()) {
                    ViperMotor.setTargetPosition(specimenPos);
                    ViperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                return true;
            }
        }
    }
    public Action slideSpecimen() {
        return new SlideSpecimen();
    }


    public class SlideReset implements Action
    {
        private boolean initialized = false;

        // actions are formatted via telemetry packets as below
        // @return true means function should rerun
        // @return false means action is complete
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {
                ViperMotor.setPower(0.8);
                initialized = true;
            }

            if (ViperMotor.getCurrentPosition() <= resetPos) {
                ViperMotor.setPower(0);
                return false;
            } else {
                if (!ViperMotor.isBusy()) {
                    ViperMotor.setTargetPosition(resetPos);
                    ViperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                return true;
            }
        }
    }
    public Action slideReset()
    {
        return new SlideReset();
    }



    public class SlideHighBar implements Action
    {
        private boolean initialized = false;

        // actions are formatted via telemetry packets as below
        // @return true means function should rerun
        // @return false means action is complete
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {
                ViperMotor.setPower(0.8);
                initialized = true;
            }

            if (ViperMotor.getCurrentPosition() <= highBarPos) {
                ViperMotor.setPower(0);
                return false;
            } else {
                if (!ViperMotor.isBusy()) {
                    ViperMotor.setTargetPosition(highBarPos);
                    ViperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                return true;
            }
        }
    }
    public Action slideHighBar()
    {
        return new SlideHighBar();
    }






























}


