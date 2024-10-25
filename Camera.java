package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
//Disabled
public class Camera extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor a = null;
    private DcMotor b = null;
    private DcMotor c = null;
    private DcMotor d = null;


    private DcMotor leftLinearSlide = null;
    private DcMotor rightLinearSlide = null;
    private DcMotor intake = null;


    private Servo airplane = null;
    private Servo leftGateLifter = null;
    private Servo rightGateLifter = null;
    private Servo magazineSpinner = null;
    private Servo magazineRelease = null;
    private Servo intakeServo = null;
    private Servo pixelPusher = null;

    public final static double OP_C = 2;

    public final static double ARM_MIN = -1.0;
    public final static double ARM_MAX = 1.0;
    final double ARM_SPEED = 0.001;

    public float leftLinearSlidePos;
    public float rightLinearSlidePos;

    private double tmp = 0;
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        tmp = x;
        x = y;
        y = yaw;
        yaw = tmp;
        double leftFrontPower    = y + x + yaw;
        double rightFrontPower   = y - x - yaw;
        double leftBackPower     = y - x + yaw;
        double rightBackPower    = y + x - yaw;
        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        a.setPower(leftFrontPower);
        c.setPower(rightFrontPower);
        b.setPower(leftBackPower);
        d.setPower(rightBackPower);
    }

    @Override
    public void runOpMode() {

        a  = hardwareMap.get(DcMotor.class, "left_back_drive");
        b  = hardwareMap.get(DcMotor.class, "left_front_drive");
        c = hardwareMap.get(DcMotor.class, "right_front_drive");
        d = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftLinearSlide = hardwareMap.get(DcMotor.class, "left_linear_slide");
        rightLinearSlide = hardwareMap.get(DcMotor.class, "right_linear_slide");

        intake = hardwareMap.get(DcMotor.class, "intake");
        intakeServo = hardwareMap.get(Servo.class, "intake_spin");

        airplane = hardwareMap.get(Servo.class, "airplane_release");
        leftGateLifter = hardwareMap.get(Servo.class, "left_gate_lifter");
        rightGateLifter = hardwareMap.get(Servo.class, "right_gate_lifter");
        magazineSpinner = hardwareMap.get(Servo.class, "spin");
        magazineRelease = hardwareMap.get(Servo.class, "release");
        pixelPusher = hardwareMap.get(Servo.class, "pusher");


        double armPosition = 0.5;
        double lifterPosition = 0.5;

        boolean intakeDown = false;
        boolean intakeOn = false;
        boolean stationaryMag = true;

        a.setDirection(DcMotor.Direction.FORWARD);
        b.setDirection(DcMotor.Direction.FORWARD);
        c.setDirection(DcMotor.Direction.REVERSE);
        d.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        
        float a1 = a.getCurrentPosition();
        float b1 = a.getCurrentPosition();
        float c1 = a.getCurrentPosition();
        float d1 = a.getCurrentPosition();
        telemetry.addData("%4.1f", a1);
        telemetry.update();
        while (opModeIsActive()) {//        Part 2: Go forward and putdown pixel nearby
            float aPos = a.getCurrentPosition();
            float bPos = b.getCurrentPosition();
            float cPos = c.getCurrentPosition();
            float dPos = d.getCurrentPosition();
            telemetry.addData("%4.2f", a1-aPos);
            telemetry.addData("%4.2f", b1-bPos);
            telemetry.addData("%4.2f", c1-cPos);
            telemetry.addData("%4.2f", d1-dPos);
            telemetry.update();
            moveRobot(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }



    }}
