package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Arm;
import org.firstinspires.ftc.teamcode.mechanisms.Claw;
import org.firstinspires.ftc.teamcode.mechanisms.SlideySlide;

@Autonomous(name = "RoadRunner Basket Auto", group = "Autonomous")
public final class RoadRunnerBasketAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(50, 72, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        Arm arm = new Arm(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        SlideySlide slide = new SlideySlide(hardwareMap);
        RevBlinkinLedDriver blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");;
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);


        // Actions to run on init
        Actions.runBlocking(claw.homePosition());

        waitForStart();

        if (isStopRequested()) return;

        // Move forward to submersible
        TrajectoryActionBuilder moveAndFaceBasket = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(60, 65))
                .turnTo(Math.PI / 4);

        // Move away from submersible
        // push 3 samples on field into observation zone
        TrajectoryActionBuilder moveTowardsBasket = drive.actionBuilder(new Pose2d(55, 53, Math.PI / 4))
                .strafeTo(new Vector2d(62, 62));

        Actions.runBlocking(
                new SequentialAction(
                        moveAndFaceBasket.build(),
                        arm.armUpBasket(),
                        claw.wristSpecimen(),
                        slide.slideHighBasket(),
                        //moveTowardsBasket.build(),
                        claw.openClaw()
                )
        );
    }
}
