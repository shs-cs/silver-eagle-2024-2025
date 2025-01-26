package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
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

@Autonomous(name = "Road Runner Auto", group = "Autonomous")
public final class RoadRunnerAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 72, -Math.PI / 2);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        Arm arm = new Arm(hardwareMap);
        Claw claw = new Claw(hardwareMap);


        Actions.runBlocking(claw.homePosition());


        waitForStart();

        if (isStopRequested()) return;

        TrajectoryActionBuilder moveForward = drive.actionBuilder(beginPose)
                .waitSeconds(1)
                .strafeTo(new Vector2d(0, 42));

        TrajectoryActionBuilder moveBack = drive.actionBuilder(new Pose2d(0, 42, -Math.PI / 2))
                .strafeTo(new Vector2d(0, 50))
                .strafeTo(new Vector2d(-36, 50))
                .strafeTo(new Vector2d(-36, 14))
                .strafeTo(new Vector2d(-50, 14))
                .strafeTo(new Vector2d(-50, 64))
                .strafeTo(new Vector2d(-50, 14))
                .strafeTo(new Vector2d(-53, 14))
                .strafeTo(new Vector2d(-53, 64))
                .strafeTo(new Vector2d(-57, 14))
                .strafeTo(new Vector2d(-57, 64))
                ;

        Actions.runBlocking(
                new SequentialAction(
                        arm.armUp(),
                        claw.wristSpecimen(),
                        moveForward.build(),
                        claw.openClaw(),
                        moveBack.build()
                )
        );


//        TrajectoryActionBuilder moveToBlocks = moveForward.endTrajectory().fresh()
//                .strafeTo(new Vector2d(0, 50))
//                .waitSeconds(1)
//                .setTangent(Math.PI)
//                .splineToLinearHeading(new Pose2d(-44, 0, Math.PI / 2), -Math.PI / 1.65);

//        Actions.runBlocking(
//                drive.actionBuilder(beginPose)
//                        .strafeTo(new Vector2d(-44, 65))
//                        .strafeTo(new Vector2d(-49.2, 0))
//                        .strafeTo(new Vector2d(-49.2, 65))
//                        .build());



    }
}
