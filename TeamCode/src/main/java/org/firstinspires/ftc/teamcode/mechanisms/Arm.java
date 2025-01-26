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
public class Arm {
    private DcMotorEx arm;

    public Arm(HardwareMap hardwareMap) {
        arm = hardwareMap.get(DcMotorEx.class, "armMotor");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    private static final int highBarPosition = -2900;
    private static final int highBasketPosition = -4231;

    public class ArmUp implements Action {
        private boolean initialized = false;

        // actions are formatted via telemetry packets as below
        // @return true means function should rerun
        // @return false means action is complete
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {
                arm.setPower(0.8);
                initialized = true;
            }

            if (arm.getCurrentPosition() <= highBarPosition) {
                arm.setPower(0);
                return false;
            } else {
                    arm.setTargetPosition(highBarPosition);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                return true;
            }
        }
    }
    public Action armUp() {
        return new ArmUp();
    }

    public class ArmDown implements Action {
        // actions are formatted via telemetry packets as below
        // @return true means function should rerun
        // @return false means action is complete
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (arm.getCurrentPosition() == 0) {
                arm.setPower(0.0);
                return false;
            } else {
                arm.setTargetPosition(0);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                return true;
            }
        }
    }
    public Action armDown() {
        return new ArmDown();
    }

    public class ArmUpBasket implements Action {
        private boolean initialized = false;

        // actions are formatted via telemetry packets as below
        // @return true means function should rerun
        // @return false means action is complete
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {
                arm.setPower(0.8);
                initialized = true;
            }

            if (arm.getCurrentPosition() <= highBasketPosition) {
                arm.setPower(0);
                return false;
            } else {
                arm.setTargetPosition(highBasketPosition);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                return true;
            }
        }
    }
    public Action armUpBasket() {
        return new ArmUpBasket();
    }
}


