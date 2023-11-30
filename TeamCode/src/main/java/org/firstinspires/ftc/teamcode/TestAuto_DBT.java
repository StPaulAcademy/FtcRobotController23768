/*
Copyright (c) 2023 FIRST

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;


//@Disabled
@TeleOp(name="TestAuto_DBT", group="Linear OpMode")
public class TestAuto_DBT extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor conveyerDrive = null;
    private final int READ_PERIOD = 1;
    private HuskyLens huskyLens;

    @Override
    public void runOpMode() {

        //huskyLens
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
        rateLimit.expire();

        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_TRACKING);

        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (!rateLimit.hasExpired()) {
                continue;
            }
            rateLimit.reset();

            HuskyLens.Block[] objects = huskyLens.blocks();
            telemetry.addData("Object count", objects.length);
            telemetry.addLine("Checking if in middle...");
            telemetry.update();
            sleep(5000);
            objects = huskyLens.blocks();
            if (objects.length >= 1) {
                middlePosition();
            } else {
                //strafe left, drive backwards
                setMotorPower(-.1,.1,.1,-.1, 1000);
                setMotorPower(-.1,-.1,-.1,-.1, 1000);
                telemetry.addLine("Checking if in left...");
                telemetry.update();
                sleep(5000);
                objects = huskyLens.blocks();
                if (objects.length >= 1) {
                    leftPosition();
                } else {
                    telemetry.addLine("Assuming it is right");
                    telemetry.update();
                    sleep((2000));
                    rightPosition();
                }
            }
            for (int i = 0; i < objects.length; i++) {
                telemetry.addData("Object", objects[i].toString());
                telemetry.addData("X Position", objects[i].x);
                telemetry.addData("Y Position", objects[i].y);

            }

            telemetry.update();

            //motor






        }
    }
    public void leftPosition() {
        telemetry.addLine("Left");
        telemetry.update();
        //strafe left
        setMotorPower(-.1,.1,.1,-.1, 5000);
        sleep(100000);
    }
    public void rightPosition() {
        telemetry.addLine("Right");
        telemetry.update();
        //drive forward, strafe left
        setMotorPower(.1,.1,.1,.1, 1000);
        setMotorPower(-.1,.1,.1,-.1, 5000);
        sleep(100000);
    }
    public void middlePosition() {
        telemetry.addLine("Middle");
        telemetry.update();
        //strafe left
        setMotorPower(-.1,.1,.1,-.1, 5000);
        sleep(100000);
    }

    public void setMotorPower(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower, long time) {
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
        sleep(time);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
}
