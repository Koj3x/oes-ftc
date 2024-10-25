package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="BasicLinear", group="Linear OpMode")
//Disabled
public class Slide_Test extends LinearOpMode {

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotor leftSlideDrive = null;
    private DcMotor rightSlideDrive = null;

    private ElapsedTime runtime = new ElapsedTime();
    private CRServo intake = null;

    private int slideOut = 1000;
    private int slideIn = 0;

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

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    @Override
    public void runOpMode() {

        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        
        intake = hardwareMap.get(CRServo.class, "intake");        

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftSlideDrive = hardwareMap.get(DcMotor.class, "left_slide");
        rightSlideDrive = hardwareMap.get(DcMotor.class, "right_slide");
        
        leftSlideDrive.setDirection(DcMotor.Direction.REVERSE);
        leftSlideDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightSlideDrive.setDirection(DcMotor.Direction.REVERSE);
        rightSlideDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);  

        int leftSlidePosition = leftSlideDrive.getCurrentPosition();
        int rightSlidePosition = rightSlideDrive.getCurrentPosition();

        int leftSlidePositionMax = 10;
        int rightSlidePositionMax = 10;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        while (opModeIsActive()) {
            moveRobot(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }

        if (gamepad1.a) {
            intake.setPower(1);
            telemetry.addData("servo move?");
        }

        if (gamepad1.b) {
            intake.setPower(-1);
            telemetry.addData("servo other move");
        }

        if (gamepad1.x) {
            leftSlideDrive.setTargetPosition(slideOut);
            rightSlideDrive.setTargetPosition(slideOut); 

            leftSlideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftSlideDrive.setPower(0.5);
            rightSlideDrive.setPower(0.5);

            telemetry.addData("Moving to position: " + slideOut);
        }
        
        if (gamepad1.y) {
            leftSlideDrive.setTargetPosition(slideIn);
            rightSlideDrive.setTargetPosition(slideIn); 

            leftSlideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlideDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftSlideDrive.setPower(0.5);
            rightSlideDrive.setPower(0.5);
            
            telemetry.addData("Moving to position: " + slideIn);
        }

        // Once target is reached, motor will automatically stop.
        if (!leftSlideDrive.isBusy()) {
            leftSlideDrive.setPower(0);
            telemetry.addData("left reached");
        }

        if (!rightSlideDrive.isBusy()) {
            rightSlideDrive.setPower(0);
            telemetry.addData("right reached");
        }

        waitForStart();
        while (true){
            telemetry.addData("Slide at %7d", slideDrive.getCurrentPosition());
            telemetry.update();
        }
    }
    
    @Override
    public void stop() {
        leftSlideDrive.setPower(0);
        rightSlideDrive.setPower(0);
        
        intake.setPower(0);
    }
}


