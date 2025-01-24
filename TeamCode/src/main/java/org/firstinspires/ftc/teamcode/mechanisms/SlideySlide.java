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
    private static final int highBarPosition = -3422;
    public class SlideHighBasket implements Action {
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
    public Action slideHighBasket() {
        return new SlideHighBasket();
    }
}


