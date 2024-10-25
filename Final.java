/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Basic: MainT", group="Linear OpMode")
//Disabled
public class Final extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;


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
    public void moveRobot(double speed, double rft, double rbt, double lft, double lbt,double timeoutS){
        int nRFT;
        int nRBT;
        int nLFT;
        int nLBT;
        if (opModeIsActive()) {
            nRFT = rightFrontDrive.getCurrentPosition() + (int)(rft);
            nRBT = rightBackDrive.getCurrentPosition() + (int)(rbt);
            nLFT = leftFrontDrive.getCurrentPosition() + (int)(lft);
            nLBT = leftBackDrive.getCurrentPosition() + (int)(lbt);
            rightFrontDrive.setTargetPosition(nRFT);
            rightBackDrive.setTargetPosition(nRBT);
            leftFrontDrive.setTargetPosition(nLFT);
            leftBackDrive.setTargetPosition(nLBT);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();
            rightFrontDrive.setPower(Math.abs(speed));
            rightBackDrive.setPower(Math.abs(speed));
            leftFrontDrive.setPower(Math.abs(speed));
            leftBackDrive.setPower(Math.abs(speed));
            while (opModeIsActive() && (runtime.seconds() < timeoutS) &&
                    (rightFrontDrive.isBusy() && rightBackDrive.isBusy()) && (leftFrontDrive.isBusy()
                    && leftBackDrive.isBusy())) {
                telemetry.addData("Running to", " %7d :%7d :%7d :%7d", nRFT, nRBT, nLFT, nLBT);
                telemetry.update();
            }
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    @Override
    public void runOpMode() {

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

//        Part 1 Choose phrase
        int part = 1;

//        Part 2: Go forward and putdown pixel nearby
        moveRobot(0.5, -1229, -1340, 1307, 1329, 1);

        }
    }
