package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@Autonomous(name="MeetTwoAuto_Red", group="Linear OpMode")
public class MeetTwoAuto_Red extends LinearOpMode {
  private ElapsedTime runtime = new ElapsedTime();
  private DcMotor leftFrontDrive = null;
  private DcMotor leftBackDrive = null;
  private DcMotor rightFrontDrive = null;
  private DcMotor rightBackDrive = null;
  private final int READ_PERIOD = 1;
  private HuskyLens huskyLens;
  private CRServo outtakeServo;
  private Servo clawServo;
  private DcMotor armDrive;

  @Override
  public void runOpMode() throws InterruptedException {
    // Find a motor in the hardware map named "Arm Motor"
    leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFrontDrive");
    leftBackDrive  = hardwareMap.get(DcMotor.class, "leftBackDrive");
    rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
    rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");

    armDrive = hardwareMap.get(DcMotor.class, "armDrive");
    outtakeServo = hardwareMap.crservo.get("randomServo");
    clawServo = hardwareMap.servo.get("clawServo");

    // Reset the motor encoder so that it reads zero ticks
    // Turn the motor back on, required if you use STOP_AND_RESET_ENCODER
    leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    //huskylens

    huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

    Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
    rateLimit.expire();

    if (!huskyLens.knock()) {
      telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
    } else {
      telemetry.addData(">>", "Press start to continue");
    }

    huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

    telemetry.update();

    waitForStart();

      double CPR = 537.6; // Counts per revolution
      double diameter = 3.77953; // Replace with your wheel/spool's diameter
      double circumference = Math.PI * diameter;

      // Get the current position of the motor
      float position = leftBackDrive.getCurrentPosition();
      double revolutions = position/CPR;
      double angle = revolutions * 360;
      double angleNormalized = angle % 360;
      double distance = circumference * revolutions;

      leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
      leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
      rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
      rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

      //BEGIN MOVING

    HuskyLens.Block[] objects = huskyLens.blocks();
    telemetry.addData("Object count", objects.length);
    telemetry.addData("X Position", objects[0].x);
    telemetry.addLine("Checking if in middle...");
    telemetry.update();
    sleep(1000);
    objects = huskyLens.blocks();
    if (objects.length >= 1 && objects[0].x <= 175 && objects[0].x >= 55 && objects[0].y <= 170) {
      middlePosition();
    } else {
      //strafeLeftToPosition();
      telemetry.addLine("Checking if in right...");
      telemetry.update();
      sleep(1000);
      objects = huskyLens.blocks();
      if (objects.length >= 1 && objects[0].x <= 266 && objects[0].x >= 220 && objects[0].y <= 170) {
        //rightPosition();
        middlePosition();
      } else {
        telemetry.addLine("Assuming it is left");
        telemetry.update();
        sleep((1000));
        //leftPosition();
        middlePosition();
      }
    }
/*    for (int i = 0; i < objects.length; i++) {
      telemetry.addData("Object", objects[i].toString());
      telemetry.addData("X Position", objects[i].x);
      telemetry.addData("Y Position", objects[i].y);

    } */

/*      while(opModeIsActive()) {

         CPR = 537.6; // Counts per revolution
         diameter = 3.77953; // Replace with your wheel/spool's diameter
         circumference = Math.PI * diameter;

        // Get the current position of the motor
         position = leftBackDrive.getCurrentPosition();
         revolutions = position/CPR;
         angle = revolutions * 360;
         angleNormalized = angle % 360;
         distance = circumference * revolutions;

      telemetry.addLine("leftBackDrive:");
      telemetry.addData("Encoder Position", position);
      telemetry.addData("Encoder Revolutions", revolutions);
      telemetry.addData("Encoder Angle (Degrees)", angle);
      telemetry.addData("Encoder Angle - Normalized (Degrees)", angleNormalized);
      telemetry.addData("Linear Distance", distance);
      telemetry.update();

    } */
  }

  public void runToPosition(int targetDistanceInInches) {
    double CPR = 537.6; // Counts per revolution
    double diameter = 3.77953; // Replace with your wheel/spool's diameter
    double circumference = Math.PI * diameter;
    int targetPosition = (int) ((targetDistanceInInches / circumference) * CPR);

    // Set the target position for the motor
    leftBackDrive.setTargetPosition(targetPosition);
    leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    leftFrontDrive.setPower(0.2);
    rightFrontDrive.setPower(0.2);
    leftBackDrive.setPower(0.2);
    rightBackDrive.setPower(0.2);

    while(opModeIsActive() && leftBackDrive.isBusy()) {
      telemetry.addData("Target Position", targetPosition);
      telemetry.addData("Current Position", leftFrontDrive.getCurrentPosition());
      telemetry.update();
    }

    leftFrontDrive.setPower(0);
    rightFrontDrive.setPower(0);
    leftBackDrive.setPower(0);
    rightBackDrive.setPower(0);

    leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  }

  public void strafeLeftToPosition(int targetDistanceInInches) {
    double CPR = 537.6; // Counts per revolution
    double diameter = 3.77953; // Replace with your wheel/spool's diameter
    double circumference = Math.PI * diameter;
    int targetPosition = (int) ((targetDistanceInInches / circumference) * CPR);

    // Set the target position for the motor
    leftBackDrive.setTargetPosition(targetPosition);
    leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    leftFrontDrive.setPower(-0.2);
    rightFrontDrive.setPower(0.2);
    leftBackDrive.setPower(0.2);
    rightBackDrive.setPower(-0.2);

    while(opModeIsActive() && leftBackDrive.isBusy()) {
      telemetry.addData("Target Position", targetPosition);
      telemetry.addData("Current Position", leftFrontDrive.getCurrentPosition());
      telemetry.update();
    }

    leftFrontDrive.setPower(0);
    rightFrontDrive.setPower(0);
    leftBackDrive.setPower(0);
    rightBackDrive.setPower(0);

    leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  }

  public void strafeRightToPosition(int targetDistanceInInches) {
    double CPR = 537.6; // Counts per revolution
    double diameter = 3.77953; // Replace with your wheel/spool's diameter
    double circumference = Math.PI * diameter;
    int targetPosition = (int) ((targetDistanceInInches / circumference) * CPR * -1);

    // Set the target position for the motor
    leftBackDrive.setTargetPosition(targetPosition);
    leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    leftFrontDrive.setPower(0.2);
    rightFrontDrive.setPower(-0.2);
    leftBackDrive.setPower(-0.2);
    rightBackDrive.setPower(0.2);

    while(opModeIsActive() && leftBackDrive.isBusy()) {
      telemetry.addData("Target Position", targetPosition);
      telemetry.addData("Current Position", leftFrontDrive.getCurrentPosition());
      telemetry.update();
    }

    leftFrontDrive.setPower(0);
    rightFrontDrive.setPower(0);
    leftBackDrive.setPower(0);
    rightBackDrive.setPower(0);

    leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

  public void middlePosition() {
    //place pixel
    runToPosition(33);
    outtakeServo.setPower(-1);
    clawServo.setPosition(-1);
    armDrive.setPower(.5);
    sleep(1000);
    armDrive.setPower(0);
    runToPosition(25);
    strafeRightToPosition(120);
    setMotorPower(-.3,-.3,-.3,-.3,1000);
    sleep(10000000);
  }

  public void leftPosition() {
    runToPosition(53);
    strafeRightToPosition(115);
    setMotorPower(-.3,-.3,-.3,-.3,1000);
    //place pixel
    /*runToPosition(27);
    strafeRightToPosition(68);
    runToPosition(40);
    strafeRightToPosition(40);*/
    sleep(10000000);
  }

  public void rightPosition() {
    //place pixel
    runToPosition(26);
    strafeRightToPosition(9);
    outtakeServo.setPower(-1);
    clawServo.setPosition(-1);
    armDrive.setPower(.5);
    sleep(1000);
    armDrive.setPower(0);
    runToPosition(25);
    strafeRightToPosition(120);
    setMotorPower(-.3,-.3,-.3,-.3,1000);
    sleep(10000000);
  }
}