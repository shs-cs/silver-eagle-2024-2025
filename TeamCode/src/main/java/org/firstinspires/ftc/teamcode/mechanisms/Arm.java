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
// lift class
public class Arm {
    private DcMotorEx arm;

    public Arm(HardwareMap hardwareMap) {
        arm = hardwareMap.get(DcMotorEx.class, "armMotor");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }
    private static final int ticks = 537;
    private static final double gear_ratio = 1.0;
    private static final double wheelDiameter = 4.0;
    private static final double countsPerInch = (ticks * gear_ratio) / (Math.PI * wheelDiameter);
    public class ArmUp implements Action {
        // checks if the lift motor has been powered on
        private boolean initialized = false;

        // actions are formatted via telemetry packets as below
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // powers on motor, if it is not on
            if (!initialized) {
                arm.setPower(0.8);
                initialized = true;
            }

            int targetPosition = (int) (-80 * countsPerInch);
            if (arm.getCurrentPosition() <= -80 * countsPerInch) {
                arm.setPower(0);
                return false;
            } else {
                if (!arm.isBusy()) {
                    arm.setTargetPosition(targetPosition);
                }
                return true;
            }
        }
    }
    public Action armUp() {
        return new ArmUp();
    }
}


