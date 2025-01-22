package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "MoveMultipleThingsWithTimerTest", group = "Autonomous")

public class MoveMultipleThingsWithTimerTest extends LinearOpMode{
    //setting up motors and servos for use + Setting up Positions 
    public int ArmHighBasketPosition = -19;
    public int ViperSlideSpecimenPosition = -25;
    public int ArmSpecimenPosition = -80;
    public double WristGrabbingPosition = 0.0;
    public double ClawOpenNormalPosition =  0.37;
    public double ClawOpenWidePosition = 0.4;
    public double ClawNomNom = 0.25;
    public double WristRestPosition = 0.92;
    public double BackwardsWristHighBasketPosition = 0.7;
    public double WristSpecimenPosition = 0.32;
    public double WristHighBasketPosition = 0.32;
    public double TwistyTurnyWristStraight = 0.675;
    public double TwistyTurnyFlipPosition = 0.0;
    private static final long TimeOutLength = 2000; //timer in milliseconds
    private DcMotor LeftFront;
    private DcMotor RightFront;
    private DcMotor LeftRear;
    private DcMotor RightRear;
    private DcMotor LeftArmMotor;
    private DcMotor RightArmMotor;
    private DcMotor ViperMotor;
    private Servo LeftGripperServo;
    private Servo RightGripperServo;
    private Servo WristServo;
    private Servo TwistyTurnyServo;

    private static final int ticks = 537;
    private static final double gear_ratio = 1.0;
    private static final double wheelDiameter = 4.0; // Wheel diameter in inches

    //private static final double countsPerInch = ((ticks * gear_ratio )/(Math.PI *1.5));
    private static final double countsPerInch = (ticks * gear_ratio) / (Math.PI * wheelDiameter);


    @Override
    public void runOpMode() {
        // Initializing hardware to driver hub
        LeftFront = hardwareMap.dcMotor.get("leftFront");
        RightFront = hardwareMap.dcMotor.get("rightFront");
        LeftRear = hardwareMap.dcMotor.get("leftBack");
        RightRear = hardwareMap.dcMotor.get("rightBack");
        ViperMotor = hardwareMap.dcMotor.get("ViperMotor");
        LeftGripperServo = hardwareMap.get(Servo.class, "LeftGripperServo");
        WristServo = hardwareMap.get(Servo.class, "WristServo");
        RightGripperServo = hardwareMap.get(Servo.class, "RightGripperServo");
        TwistyTurnyServo = hardwareMap.get(Servo.class, "TwistyTurnyServo");
        LeftArmMotor = hardwareMap.dcMotor.get("LeftArmMotor");
        RightArmMotor = hardwareMap.dcMotor.get("RightArmMotor");


        // Reverse the direction of the correct motors and servos
        RightFront.setDirection(DcMotor.Direction.REVERSE);
        RightRear.setDirection(DcMotor.Direction.REVERSE);
        LeftRear.setDirection(DcMotor.Direction.REVERSE);
        RightGripperServo.setDirection(Servo.Direction.REVERSE);
        WristServo.setDirection(Servo.Direction.REVERSE);

        // Set the motors to brake when power is set to 0
        LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ViperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset the encoders of all the motors
        RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ViperMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ViperMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


//**************************** Put Initialization Code Under Here 0_0 ******************************

        TwistyTurnyServo.setPosition(TwistyTurnyWristStraight);
        clawClose();
        WristServo.setPosition(WristRestPosition);

        /*telemetry.addData("Right Front Encoder", RightFront.getCurrentPosition());
        telemetry.addData("Left Front Encoder", LeftFront.getCurrentPosition());
        telemetry.addData("Right Rear Encoder", RightRear.getCurrentPosition());
        telemetry.addData("Left Rear Encoder", LeftRear.getCurrentPosition());
        telemetry.addData("Left Arm Encoder", LeftArmMotor.getCurrentPosition());
        telemetry.addData("Right Arm Encoder", RightArmMotor.getCurrentPosition());
        telemetry.addData("Viper Slide Encoder", ViperMotor.getCurrentPosition());
        telemetry.update();

         */


        waitForStart();
//***************************** Put Auto Code Under Here To Run :o (please work) *******************

        ScoreStartingSpecimen();
        ResetWheelEnoders();

        StrafeLeftTiles(0.6, 3.0);
        Pause(100);

        TurnRightDegrees(1.0, 20);
        Pause(500);
        ResetEncoders();

        clawOpenWide();

        MoveWrist(WristGrabbingPosition);

        MovebotAndMoveSlideAndRaiseArm(1.0,-15, 1.0, 20, 1.0, 35);

        clawClose();
        MoveWrist(BackwardsWristHighBasketPosition);
        ResetEncoders();
        Pause(200);

        TurnRightDegrees(1.0,30);
        ResetWheelEnoders();

        MovebotAndRaiseArmAndMoveSlide(1.0,12,1.0,-100, 1.0, -50 );



//********************************** End of Auto :( (we're doomed) *********************************
    }
//************************************ Autonomous Functions :) *************************************

    public void Pause(int MilliSeconds) //cooler version of sleep :0 (holy moly)
    {
        stopMotors();
        sleep(MilliSeconds);
        stopMotors();
    }

    public void ResetEncoders()
    {
        RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ViperMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void StrafeLeftTiles(double Power, double TileCount)
    {
        StrafeLeftPosition(Power, (int)(-24 * TileCount));
    }

    public void StrafeRightTiles(double Power, double TileCount)
    {
        StrafeRightPosition(Power, (int)(24 * TileCount));
    }
    public void MoveTiles(double Power, double TileCount)
    {
        MovePosition(Power, (int)(-24 * TileCount));
    }
    public void TurnLeftDegrees(double power, int degrees)
    { //chat gbt idubidubly assisted me (idk how to spell that)

        // Number of encoder counts for a 90 degree turn
        int basePositionFor90Degrees = 25;
        // Calculate the target position based on the number of degrees to turn
        int targetPosition = (int) (basePositionFor90Degrees * (degrees / 90.0));
        // Call the TurnPosition method with the calculated target position
        TurnPosition(power, targetPosition);
    }

    public void TurnRightDegrees(double power, int degrees)
    { //chat gbt idubidubly assisted me (idk how to spell that)

        // Number of encoder counts for a 90 degree turn
        int basePositionFor90Degrees = -25;
        // Calculate the target position based on the number of degrees to turn
        int targetPosition = (int) (basePositionFor90Degrees * (degrees / 90.0));
        // Call the TurnPosition method with the calculated target position
        TurnPosition(power, targetPosition);
    }

    public void StrafeTiles(double Power, double TileCount)
    {
        StrafePosition(Power, (int)(24 * TileCount));
    }

    public void ScoreStartingSpecimen()
    {
        MovebotAndRaiseArmAndMoveSlide(1.0,-20, 1.0, ArmSpecimenPosition, -1.0, ViperSlideSpecimenPosition);

        MoveWrist(WristSpecimenPosition);

        ResetWheelEnoders();
        Pause(1100);

        MoveTiles(1.0, -0.5);
    }

    public void MovebotAndRaiseArmAndMoveSlide(double MovePower, int MovePosition, double ArmPower, int ArmPosition, double ViperPower, int ViperPosition) {

            MovePosition(MovePower, MovePosition); // This moves the robot

            SetArmPosition(ArmPower, ArmPosition); // This moves the arm

            //sleep(500);

            SetViperSlidePosition(ViperPower, ViperPosition); // this moves the slide


            while (opModeIsActive())
            {
                boolean RobotFinished = !RightFront.isBusy() && !LeftFront.isBusy() && !RightRear.isBusy() && !LeftRear.isBusy();

                boolean ArmFinished = !RightArmMotor.isBusy() && !LeftArmMotor.isBusy();

                boolean ViperSlideFinished = !ViperMotor.isBusy();

                // If the robot, arm, and slide have finished doing their thing then stop
                if (RobotFinished && ArmFinished && ViperSlideFinished)
                {
                    break;
                }

                // show when the slide,arm, or robot are moving and when they stop
                if (!RobotFinished)
                {
                    telemetry.addData("The Robot is", "Moving");
                }
                else
                {    telemetry.addData("The Robot is", "Finished Moving");

                }

                if (!ArmFinished)
                {
                    telemetry.addData("The Arm is", "Moving");
                }
                else
                {
                    telemetry.addData("The Arm is", "Finished Moving");
                }

                if (!ViperSlideFinished)
                {
                    telemetry.addData("The Slide is", "Moving");
                }
                else
                {
                    telemetry.addData("The Slide is", "Finished Moving");
                }

                telemetry.update();
            }

            stopMotors();

            ResetMotorsBackToNormalRunMode();
        }

    public void StrafeLeftbotAndRaiseArmAndMoveSlide(double StrafePower, double TileCount, double ArmPower, int ArmPosition, double ViperPower, int ViperPosition) {

        StrafeLeftTiles(StrafePower, TileCount); // This moves the robot

        SetArmPosition(ArmPower, ArmPosition); // This moves the arm
        //sleep(500);
        SetViperSlidePosition(ViperPower, ViperPosition); // this moves the slide


        while (opModeIsActive())
        {

            boolean RobotFinished = !RightFront.isBusy() && !LeftFront.isBusy() && !RightRear.isBusy() && !LeftRear.isBusy();

            boolean ArmFinished = !RightArmMotor.isBusy() && !LeftArmMotor.isBusy();

            boolean ViperSlideFinished = !ViperMotor.isBusy();

            // If the robot, arm, and slide have finished doing their thing then stop
            if (RobotFinished && ArmFinished && ViperSlideFinished)
            {
                break;
            }

            // show when the slide,arm, or robot are moving and when they stop
            if (!RobotFinished)
            {
                telemetry.addData("The Robot is", "Moving");
            }
            else
            {    telemetry.addData("The Robot is", "Finished Moving");

            }

            if (!ArmFinished)
            {
                telemetry.addData("The Arm is", "Moving");
            }
            else
            {
                telemetry.addData("The Arm is", "Finished Moving");
            }

            if (!ViperSlideFinished)
            {
                telemetry.addData("The Slide is", "Moving");
            }
            else
            {
                telemetry.addData("The Slide is", "Finished Moving");
            }

            telemetry.update();
        }

        stopMotors();

        ResetMotorsBackToNormalRunMode();
    }
    public void MovebotAndMoveSlideAndRaiseArm(double MovePower, int MovePosition, double ViperPower, int ViperPosition, double ArmPower, int ArmPosition) {

        MovePosition(MovePower, MovePosition); // This moves the robot

        SetViperSlidePosition(ViperPower, ViperPosition); // this moves the slide

        SetArmPosition(ArmPower, ArmPosition); // This moves the arm


        while (opModeIsActive())
        {
            boolean RobotFinished = !RightFront.isBusy() && !LeftFront.isBusy() && !RightRear.isBusy() && !LeftRear.isBusy();

            boolean ArmFinished = !RightArmMotor.isBusy() && !LeftArmMotor.isBusy();

            boolean ViperSlideFinished = !ViperMotor.isBusy();

            // If the robot, arm, and slide have finished doing their thing then stop
            if (RobotFinished && ArmFinished && ViperSlideFinished)
            {
                break;
            }

            // show when the slide,arm, or robot are moving and when they stop
            if (!RobotFinished)
            {
                telemetry.addData("The Robot is", "Moving");
            }
            else
            {    telemetry.addData("The Robot is", "Finished Moving");

            }

            if (!ArmFinished)
            {
                telemetry.addData("The Arm is", "Moving");
            }
            else
            {
                telemetry.addData("The Arm is", "Finished Moving");
            }

            if (!ViperSlideFinished)
            {
                telemetry.addData("The Slide is", "Moving");
            }
            else
            {
                telemetry.addData("The Slide is", "Finished Moving");
            }

            telemetry.update();
        }

        stopMotors();

        ResetMotorsBackToNormalRunMode();
    }

    public void TurnLeftbotAndRaiseArmAndMoveSlide(double TurnPower, int Degrees, double ArmPower, int ArmPosition, double ViperPower, int ViperPosition) {

        TurnLeftDegrees(TurnPower, Degrees); // This moves the robot

        SetArmPosition(ArmPower, ArmPosition); // This moves the arm
        sleep(500);
        SetViperSlidePosition(ViperPower, ViperPosition); // this moves the slide


        while (opModeIsActive())
        {

            boolean RobotFinished = !RightFront.isBusy() && !LeftFront.isBusy() && !RightRear.isBusy() && !LeftRear.isBusy();

            boolean ArmFinished = !RightArmMotor.isBusy() && !LeftArmMotor.isBusy();

            boolean ViperSlideFinished = !ViperMotor.isBusy();

            // If the robot, arm, and slide have finished doing their thing then stop
            if (RobotFinished && ArmFinished && ViperSlideFinished)
            {
                break;
            }

            // show when the slide,arm, or robot are moving and when they stop
            if (!RobotFinished)
            {
                telemetry.addData("The Robot is", "Moving");
            }
            else
            {    telemetry.addData("The Robot is", "Finished Moving");

            }

            if (!ArmFinished)
            {
                telemetry.addData("The Arm is", "Moving");
            }
            else
            {
                telemetry.addData("The Arm is", "Finished Moving");
            }

            if (!ViperSlideFinished)
            {
                telemetry.addData("The Slide is", "Moving");
            }
            else
            {
                telemetry.addData("The Slide is", "Finished Moving");
            }

            telemetry.update();
        }

        stopMotors();

        ResetMotorsBackToNormalRunMode();
    }

    public void TurnRightbotAndRaiseArmAndMoveSlide(double TurnPower, int Degrees, double ArmPower, int ArmPosition, double ViperPower, int ViperPosition) {

        TurnRightDegrees(TurnPower, Degrees); // This moves the robot

        SetArmPosition(ArmPower, ArmPosition); // This moves the arm
        sleep(500);
        SetViperSlidePosition(ViperPower, ViperPosition); // this moves the slide


        while (opModeIsActive())
        {

            boolean RobotFinished = !RightFront.isBusy() && !LeftFront.isBusy() && !RightRear.isBusy() && !LeftRear.isBusy();

            boolean ArmFinished = !RightArmMotor.isBusy() && !LeftArmMotor.isBusy();

            boolean ViperSlideFinished = !ViperMotor.isBusy();

            // If the robot, arm, and slide have finished doing their thing then stop
            if (RobotFinished && ArmFinished && ViperSlideFinished)
            {
                break;
            }

            // show when the slide,arm, or robot are moving and when they stop
            if (!RobotFinished)
            {
                telemetry.addData("The Robot is", "Moving");
            }
            else
            {    telemetry.addData("The Robot is", "Finished Moving");

            }

            if (!ArmFinished)
            {
                telemetry.addData("The Arm is", "Moving");
            }
            else
            {
                telemetry.addData("The Arm is", "Finished Moving");
            }

            if (!ViperSlideFinished)
            {
                telemetry.addData("The Slide is", "Moving");
            }
            else
            {
                telemetry.addData("The Slide is", "Finished Moving");
            }

            telemetry.update();
        }

        stopMotors();

        ResetMotorsBackToNormalRunMode();
    }


    private void ResetMotorsBackToNormalRunMode()
    {
        RightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ViperMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void SetArmPosition(double power, int position)
    {
        SetArmPosition(power, position, TimeOutLength);
    }

    public void SetArmPosition(double power, int position, long timeoutMillis)
    {
        int targetPosition = (int) (position * countsPerInch);

        RightArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RightArmMotor.setTargetPosition(targetPosition);
        LeftArmMotor.setTargetPosition(targetPosition);

        RightArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        RightArmMotor.setPower(power);
        LeftArmMotor.setPower(power);

        long startTime = System.currentTimeMillis();
        while (opModeIsActive() && (RightArmMotor.isBusy() || LeftArmMotor.isBusy()) &&
                (System.currentTimeMillis() - startTime < timeoutMillis)) {
            telemetry.addData("Right Arm Position", RightArmMotor.getCurrentPosition());
            telemetry.addData("Left Arm Position", LeftArmMotor.getCurrentPosition());
            telemetry.update();
        }

        RightArmMotor.setPower(0);
        LeftArmMotor.setPower(0);

        RightArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // Overloaded method for SetViperSlidePosition with default timeout
    public void SetViperSlidePosition(double power, int position) {
        SetViperSlidePosition(power, position, TimeOutLength);
    }

    public void SetViperSlidePosition(double power, int position, long timeoutMillis) {
        int targetPosition = (int) (position * countsPerInch);
        double Correction = 1.00;
        targetPosition *= Correction;
        int FinalTargetPosition = (int) targetPosition;

        ViperMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ViperMotor.setTargetPosition(FinalTargetPosition);
        ViperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ViperMotor.setPower(power);

        long startTime = System.currentTimeMillis();
        while (opModeIsActive() && ViperMotor.isBusy() &&
                (System.currentTimeMillis() - startTime < timeoutMillis)) {
            telemetry.addData("Viper Motor Position", ViperMotor.getCurrentPosition());
            telemetry.update();
        }

        ViperMotor.setPower(0);
        ViperMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // Overloaded method for MovePosition with default timeout
    public void MovePosition(double power, int position) {
        MovePosition(power, position, TimeOutLength);
    }

    public void MovePosition(double power, int position, long timeoutMillis) {
        int targetPosition = (int) (position * countsPerInch);
        double Correction = 1.00;
        targetPosition *= Correction;
        int FinalTargetPosition = (int) targetPosition;

        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RightFront.setTargetPosition(FinalTargetPosition);
        LeftFront.setTargetPosition(FinalTargetPosition);
        RightRear.setTargetPosition(FinalTargetPosition);
        LeftRear.setTargetPosition(FinalTargetPosition);

        RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        RightFront.setPower(power + 0.2);
        LeftFront.setPower(power);
        RightRear.setPower(power);
        LeftRear.setPower(power);

        long startTime = System.currentTimeMillis();
        while (opModeIsActive() && (RightFront.isBusy() || LeftFront.isBusy() || RightRear.isBusy() || LeftRear.isBusy()) &&
                (System.currentTimeMillis() - startTime < timeoutMillis)) {
            telemetry.addData("Right Front Position", RightFront.getCurrentPosition());
            telemetry.addData("Left Front Position", LeftFront.getCurrentPosition());
            telemetry.addData("Right Rear Position", RightRear.getCurrentPosition());
            telemetry.addData("Left Rear Position", LeftRear.getCurrentPosition());
            telemetry.update();
        }

        RightFront.setPower(0);
        LeftFront.setPower(0);
        RightRear.setPower(0);
        LeftRear.setPower(0);

        ResetWheelEnoders();

        RightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // Overloaded method for TurnPosition with default timeout
    public void TurnPosition(double power, int position) {
        TurnPosition(power, position, TimeOutLength);
    }

    public void TurnPosition(double power, int position, long timeoutMillis) {
        int targetPosition = (int) (position * countsPerInch);
        double Correction = 1.01;
        targetPosition *= Correction;
        int FinalTargetPosition = (int) targetPosition;

        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RightFront.setTargetPosition(-FinalTargetPosition);
        LeftFront.setTargetPosition(FinalTargetPosition);
        LeftRear.setTargetPosition(FinalTargetPosition);
        RightRear.setTargetPosition(-FinalTargetPosition);

        RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        RightFront.setPower(-power);
        LeftFront.setPower(power);
        RightRear.setPower(-power);
        LeftRear.setPower(power);

        long startTime = System.currentTimeMillis();
        while (opModeIsActive() && (RightFront.isBusy() || LeftFront.isBusy() || RightRear.isBusy() || LeftRear.isBusy()) &&
                (System.currentTimeMillis() - startTime < timeoutMillis)) {
            telemetry.addData("Right Front Position", RightFront.getCurrentPosition());
            telemetry.addData("Left Front Position", LeftFront.getCurrentPosition());
            telemetry.addData("Right Rear Position", RightRear.getCurrentPosition());
            telemetry.addData("Left Rear Position", LeftRear.getCurrentPosition());
            telemetry.update();
        }

        RightFront.setPower(0);
        LeftFront.setPower(0);
        RightRear.setPower(0);
        LeftRear.setPower(0);

        ResetWheelEnoders();

        RightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void StrafePosition(double power, int position)
    {
        StrafePosition(power, position, TimeOutLength);
    }


    public void StrafeLeftPosition(double power, int position)
    {
        StrafeLeftPosition(power, position, TimeOutLength);
    }

    public void StrafeLeftPosition(double power, int position, long timeoutMillis) {
        int targetPosition = (int) (position * countsPerInch);
        double Correction = 1.00;
        targetPosition *= Correction;
        int FinalTargetPosition = (int) targetPosition;

        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RightFront.setTargetPosition(-FinalTargetPosition);
        LeftFront.setTargetPosition(FinalTargetPosition);
        RightRear.setTargetPosition(FinalTargetPosition);
        LeftRear.setTargetPosition(-FinalTargetPosition);

        RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        RightFront.setPower(-power);
        LeftFront.setPower(power);
        RightRear.setPower(power);
        LeftRear.setPower(-power);

        long startTime = System.currentTimeMillis();
        while (opModeIsActive() && (RightFront.isBusy() || LeftFront.isBusy() || RightRear.isBusy() || LeftRear.isBusy()) &&
                (System.currentTimeMillis() - startTime < timeoutMillis)) {
            telemetry.addData("Right Front Position", RightFront.getCurrentPosition());
            telemetry.addData("Left Front Position", LeftFront.getCurrentPosition());
            telemetry.addData("Right Rear Position", RightRear.getCurrentPosition());
            telemetry.addData("Left Rear Position", LeftRear.getCurrentPosition());
            telemetry.update();
        }

        RightFront.setPower(0);
        LeftFront.setPower(0);
        RightRear.setPower(0);
        LeftRear.setPower(0);

        ResetWheelEnoders();

        RightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }



    public void StrafePosition(double power, int position, long timeoutMillis)
    {
        int targetPosition = (int) (position * countsPerInch);
        double Correction = 1.00;
        targetPosition *= Correction;
        int FinalTargetPosition = (int) targetPosition;

        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RightFront.setTargetPosition(-FinalTargetPosition);
        LeftFront.setTargetPosition(FinalTargetPosition);
        RightRear.setTargetPosition(FinalTargetPosition);
        LeftRear.setTargetPosition(-FinalTargetPosition);

        RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        RightFront.setPower(-power);
        LeftFront.setPower(power);
        RightRear.setPower(power);
        LeftRear.setPower(-power);

        long startTime = System.currentTimeMillis();
        while (opModeIsActive() && (RightFront.isBusy() || LeftFront.isBusy() || RightRear.isBusy() || LeftRear.isBusy()) &&
                (System.currentTimeMillis() - startTime < timeoutMillis)) {
            telemetry.addData("Right Front Position", RightFront.getCurrentPosition());
            telemetry.addData("Left Front Position", LeftFront.getCurrentPosition());
            telemetry.addData("Right Rear Position", RightRear.getCurrentPosition());
            telemetry.addData("Left Rear Position", LeftRear.getCurrentPosition());
            telemetry.update();
        }

        RightFront.setPower(0);
        LeftFront.setPower(0);
        RightRear.setPower(0);
        LeftRear.setPower(0);

        ResetWheelEnoders();

        RightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void StrafeRightPosition(double power, int position) {
        StrafeRightPosition(power, position, TimeOutLength);
    }

    public void StrafeRightPosition(double power, int position, long timeoutMillis) {
        int targetPosition = (int) (position * countsPerInch);
        double Correction = 1.00;
        targetPosition *= Correction;
        int FinalTargetPosition = (int) targetPosition;

        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RightFront.setTargetPosition(FinalTargetPosition);
        LeftFront.setTargetPosition(-FinalTargetPosition);
        RightRear.setTargetPosition(-FinalTargetPosition);
        LeftRear.setTargetPosition(FinalTargetPosition);

        RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        RightFront.setPower(power);
        LeftFront.setPower(-power);
        RightRear.setPower(-power);
        LeftRear.setPower(power);

        long startTime = System.currentTimeMillis();
        while (opModeIsActive() && (RightFront.isBusy() || LeftFront.isBusy() || RightRear.isBusy() || LeftRear.isBusy()) &&
                (System.currentTimeMillis() - startTime < timeoutMillis)) {
            telemetry.addData("Right Front Position", RightFront.getCurrentPosition());
            telemetry.addData("Left Front Position", LeftFront.getCurrentPosition());
            telemetry.addData("Right Rear Position", RightRear.getCurrentPosition());
            telemetry.addData("Left Rear Position", LeftRear.getCurrentPosition());
            telemetry.update();
        }

        RightFront.setPower(0);
        LeftFront.setPower(0);
        RightRear.setPower(0);
        LeftRear.setPower(0);

        ResetWheelEnoders();

        RightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void ResetWheelEnoders()
    {
        RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void clawOpen()
    {
        LeftGripperServo.setPosition(ClawOpenNormalPosition); // Open Left Gripper
        RightGripperServo.setPosition(ClawOpenNormalPosition); // Open Right Gripper
    }

    public void clawOpenWide()
    {
        LeftGripperServo.setPosition(ClawOpenWidePosition);
        RightGripperServo.setPosition(ClawOpenWidePosition);
    }
    public void clawClose()
    {
        LeftGripperServo.setPosition(ClawNomNom); // Close left Gripper
        RightGripperServo.setPosition(ClawNomNom); // Close Right Gripper
    }
    public void MoveWrist(double pos)
    {
        WristServo.setPosition(pos);

    }
    public void stopMotors(){
        LeftFront.setPower(0);
        RightFront.setPower(0);
        LeftRear.setPower(0);
        RightRear.setPower(0);
        LeftArmMotor.setPower(0);
        RightArmMotor.setPower(0);
        ViperMotor.setPower(0);
    }
}
