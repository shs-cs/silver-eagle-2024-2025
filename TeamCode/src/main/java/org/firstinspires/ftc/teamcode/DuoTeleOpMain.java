package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name = "DuoTeleOpMain", group = "TeleOp")
public class DuoTeleOpMain extends OpMode {
    //setting up motors and servos for use + Setting up Positions
   //public double BatteryLevel = hardwareMap.voltageSensor.iterator().next().getVoltage();
    public double TwistyTurnySidePosition = 0.39;
    public double TwistyTurnyStraight = 0.75;
    public double TwistyTurnyFlipPosition = 0.08; //0.675
    public double ClawOpenNormalPos =  0.15;
    public double ClawOpenWidePos = 0.05;
    public double ClawNomNom = 0.32;

    public double WristRestPosition = 0.16;
    public double WristGrabbingPosition = 0.016;
    public double WristSlammaJammaPosition = 0.17;

    public double WristSpecimenPosition = 0.064;

    public double WristBackScoringPosition = 0.097;
    public double WristHighBasketPosition = 0.064;
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
    public void init() {
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
        RightGripperServo.setDirection(Servo.Direction.REVERSE);
        WristServo.setDirection(Servo.Direction.REVERSE);

        LeftArmMotor = hardwareMap.dcMotor.get("armMotor");
        RightArmMotor = hardwareMap.dcMotor.get("junk");
        LeftArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reverse the direction of the correct motors
            RightFront.setDirection(DcMotor.Direction.REVERSE);
        RightRear.setDirection(DcMotor.Direction.REVERSE);
       // LeftRear.setDirection(DcMotor.Direction.REVERSE);

        // Set the motors to brake when power is set to 0
        LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ViperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        // reset motor encoders
        RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ViperMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set motors back to default run mode
        ViperMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    @Override
    public void loop()
    {
        handleDriveTrain();
        ArmGoVroom();
        ViperSlideVroom();
        ServoGoVroom();

        telemetry.addData("ViperMotor Power", ViperMotor.getPower());
        telemetry.addData("ViperMotor Position", ViperMotor.getCurrentPosition());
        telemetry.addData("LeftArmMotor Position:", LeftArmMotor.getCurrentPosition());
        telemetry.addData("RightArmMotor Position:", RightArmMotor.getCurrentPosition());
        telemetry.addData("LeftArm Position", LeftArmMotor.getCurrentPosition());
        telemetry.addData("Twisty Turny Position", TwistyTurnyServo.getPosition());
        //telemetry.addData("Battery Power:", BatteryLevel);

    }

    public void handleDriveTrain()
    {
        double leftStickY = gamepad1.left_stick_y; // Y axis for forward/backward
        double leftStickX = gamepad1.left_stick_x; // Left stick X for strafe
        double rightStickX = gamepad1.right_stick_x; // Right stick X for rotation

        //Rename Stick variables For better understanding
        double strafePower = leftStickX; // Lateral movement
        double forwardPower = leftStickY * 0.8; // Forward/backward movement
        double rotationPower = rightStickX * 0.5; // Rotation

        // Calculate power for each motor
        double powerLF = getPower(forwardPower - strafePower - rotationPower);  // Left Front
        double powerRF = getPower(forwardPower + strafePower + rotationPower);  // Right Front
        double powerLR = getPower(forwardPower + strafePower - rotationPower);  // Left Rear
        double powerRR = getPower(forwardPower - strafePower + rotationPower);  // Right Rear

        // Set motor powers
        LeftFront.setPower(powerLF);
        RightFront.setPower(powerRF);
        LeftRear.setPower(powerLR);
        RightRear.setPower(powerRR);

    }

    public int ArmMidLowThreshold = 1871;
    public int ArmHighThreshold = -3060;
    public int slideMax = -1528;



      public void ViperSlideVroom()
      {

        double rStick = gamepad2.right_stick_y;
        int armPosition = LeftArmMotor.getCurrentPosition();
        int slidePosition = ViperMotor.getCurrentPosition();

        boolean armHigh = armPosition <=  ArmHighThreshold; //&& armPosition >= lowBound
        boolean armMidLow = armPosition <=  ArmMidLowThreshold;
        boolean slideAtMax = slidePosition <= slideMax; //&& slidePosition <= 0

          telemetry.addData("SlideAtMax: ", slideAtMax);

        if (gamepad2.right_stick_y < -0.1)
        {
            if(armPosition <= ArmHighThreshold)
            {
                ViperMotor.setPower(getPower(rStick));
            }

            if (armPosition > ArmHighThreshold)
            {

                if(ViperMotor.getCurrentPosition() <= slideMax)
                {
                    ViperMotor.setPower(0);
                }

                if(ViperMotor.getCurrentPosition() > slideMax)
                {
                    ViperMotor.setPower(getPower(rStick));


                }
            }

        }
        else if (gamepad2.right_stick_y > 0.1)
        {
            ViperMotor.setPower(getPower(rStick));
        }
        else if (gamepad2.right_stick_y == 0)
        {
            ViperMotor.setPower(0);
        }



    }

    public void ServoGoVroom() {
        if (gamepad2.left_bumper) {
            RightGripperServo.setPosition(ClawOpenNormalPos); // Open Right Gripper
            LeftGripperServo.setPosition(ClawOpenNormalPos); // Open Left Gripper
            //telemetry.addData("Right Gripper Position:", RightGripperServo.getPosition());
            //telemetry.addData("Left Gripper Position:", LeftGripperServo.getPosition());
            //telemetry.update();
        }
        if (gamepad2.right_bumper)
        {
            LeftGripperServo.setPosition(ClawNomNom); // Close Left Gripper
            RightGripperServo.setPosition(ClawNomNom); // Close Right Gripper
            //telemetry.addData("Right Gripper Position:", RightGripperServo.getPosition());
            //telemetry.addData("Left Gripper Position:", LeftGripperServo.getPosition());
            //telemetry.update();
        }

        if(gamepad2.dpad_left)
        {
            TwistyTurnyServo.setPosition(TwistyTurnySidePosition);
            //telemetry.addData("TwistyTurnyPosition:", TwistyTurnyServo.getPosition());
            //telemetry.update();
        }

        if(gamepad2.dpad_right)
        {
            TwistyTurnyServo.setPosition(TwistyTurnyStraight);
            //telemetry.addData("TwistyTurnyPosition:", TwistyTurnyServo.getPosition());
            //telemetry.update();
        }


        if(gamepad1.a)
        {
            TwistyTurnyServo.setPosition(TwistyTurnyStraight);
            //telemetry.addData("TwistyTurnyPosition:", TwistyTurnyServo.getPosition());
           // telemetry.update();
        }

        if(gamepad1.x)
        {
            ViperMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RightArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LeftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            ViperMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RightArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            LeftArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if(gamepad1.right_bumper)
        {

            long timeoutMillis = 750;

            long startTime = System.currentTimeMillis();

            RightFront.setPower(1);
            LeftFront.setPower(-1);
            RightRear.setPower(1);
            LeftRear.setPower(-1);

            while (System.currentTimeMillis() - startTime < timeoutMillis) {
                telemetry.addData("Turning", "Time Remaining: " + (timeoutMillis - (System.currentTimeMillis() - startTime)) + " ms");
                telemetry.update();
            }

            RightFront.setPower(0);
            LeftFront.setPower(0);
            RightRear.setPower(0);
            LeftRear.setPower(0);

            telemetry.addData("Turning", "Completed");
            telemetry.update();
        }

        if (gamepad1.left_bumper) {

            long timeoutMillis = 750;

            long startTime = System.currentTimeMillis();

            RightFront.setPower(-1);
            LeftFront.setPower(1);
            RightRear.setPower(-1);
            LeftRear.setPower(1);

            while (System.currentTimeMillis() - startTime < timeoutMillis) {
                telemetry.addData("Turning", "Time Remaining: " + (timeoutMillis - (System.currentTimeMillis() - startTime)) + " ms");
                telemetry.update();
            }

            RightFront.setPower(0);
            LeftFront.setPower(0);
            RightRear.setPower(0);
            LeftRear.setPower(0);

            telemetry.addData("Turning", "Completed");
            telemetry.update();
        }

        if(gamepad1.right_trigger > 0.5)
        {
            RightFront.setPower(1);
            LeftFront.setPower(-1);
            RightRear.setPower(-1);
            LeftRear.setPower(1);
        }

        if(gamepad1.left_trigger > 0.5)
        {
            RightFront.setPower(-1);
            LeftFront.setPower(1);
            RightRear.setPower(1);
            LeftRear.setPower(-1);
        }



        if(gamepad1.b)
        {
            TwistyTurnyServo.setPosition(TwistyTurnyFlipPosition);
            //telemetry.addData("TwistyTurnyPosition:", TwistyTurnyServo.getPosition());
            //telemetry.update();
        }
        if(gamepad2.dpad_up)
        {
            TwistyTurnyServo.setPosition(TwistyTurnyFlipPosition);
            //telemetry.addData("TwistyTurnyPosition:", TwistyTurnyServo.getPosition());
            //telemetry.update();
        }
        if (gamepad2.x)
        {
            WristServo.setPosition(WristGrabbingPosition);
            //telemetry.addData("Wrist Servo Position:", WristServo.getPosition());
            //telemetry.update();
        }
        if (gamepad2.y)
        {
            WristServo.setPosition(WristHighBasketPosition);
            //telemetry.addData("Wrist Servo Position:", WristServo.getPosition());
            //telemetry.update();
        }
        if (gamepad2.b && gamepad2.dpad_down)
        {
            WristServo.setPosition(WristRestPosition);
            //telemetry.addData("Wrist Servo Position:", WristServo.getPosition());
            //telemetry.update();
        }
        if (gamepad2.a)
        {
            WristServo.setPosition(WristBackScoringPosition);
            //telemetry.addData("Wrist Servo Position:", WristServo.getPosition());
            //telemetry.update();
        }
        if (gamepad2.left_trigger > 0.5)
        {
           LeftGripperServo.setPosition(ClawOpenWidePos);
           RightGripperServo.setPosition(ClawOpenWidePos);
           //telemetry.addData("Right Gripper Position:", RightGripperServo.getPosition());
           //telemetry.addData("Left Gripper Position:", LeftGripperServo.getPosition());
           //telemetry.update();
        }
        if (gamepad2.right_trigger > 0.5)
        {

            WristServo.setPosition(WristSlammaJammaPosition);

        }
    }

    public void ArmGoVroom() {
        double lStick = gamepad2.left_stick_y;

        int slidePosition = ViperMotor.getCurrentPosition();
        int armPosition = LeftArmMotor.getCurrentPosition();

        boolean armHigh = armPosition >=  ArmHighThreshold; //&& armPosition >= lowBound
        boolean armMidLow = armPosition <=  ArmMidLowThreshold;
        boolean slideAtMax = slidePosition >= slideMax; //&& slidePosition <= 0


            if (gamepad2.left_stick_y < -0.1)
            {

                    LeftArmMotor.setPower(getPower(-lStick));
                    RightArmMotor.setPower(getPower(-lStick));


            }

            if (gamepad2.left_stick_y > 0.1)
            {
                LeftArmMotor.setPower(getPower(-lStick));
                RightArmMotor.setPower(getPower(-lStick));


            }

            if (gamepad2.left_stick_y == 0) {
                LeftArmMotor.setPower(0);
                RightArmMotor.setPower(0);
            }




    }
    public double getPower(double powerLevel) {

        if (powerLevel > 1) {
            return 1;
        }

        else if (powerLevel < -1) {
            return -1;
        }

        return powerLevel;
    }
    public void ResetWheelEnoders()
    {
        RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    @Override
    public void stop() {
        LeftFront.setPower(0);
        RightFront.setPower(0);
        LeftRear.setPower(0);
        RightRear.setPower(0);
        LeftArmMotor.setPower(0);
        RightArmMotor.setPower(0);
        ViperMotor.setPower(0);
    }
}