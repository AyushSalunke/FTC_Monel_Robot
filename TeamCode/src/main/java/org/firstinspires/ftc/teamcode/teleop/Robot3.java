package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.motors.RevRoboticsUltraPlanetaryHdHexMotor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(group = "BumbleBees")
public class Robot3 extends LinearOpMode {
    DcMotorEx motorLift;
    DcMotorEx leftRear;
    DcMotorEx rightRear;
    Servo ServoIntakeLeft, ServoIntakeRight;
    Servo ServoGripperLeft, ServoGripperRight;
    boolean gripFlag1 = false;
    boolean gripFlag2 = false;
    public double ServoIntakeLeftPos, ServoIntakeRightPos;
    public int pos = 500;
    public int initpos = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        motorLift = hardwareMap.get(DcMotorEx.class, "motorLift");
        ServoGripperLeft = hardwareMap.servo.get("ServoGripperLeft");
        ServoGripperRight = hardwareMap.servo.get("ServoGripperRight");
        ServoIntakeLeft = hardwareMap.servo.get("ServoIntakeLeft");
        ServoIntakeRight = hardwareMap.servo.get("ServoIntakeRight");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");

        motorLift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
//        motorLift.setDirection(DcMotorSimple.Direction.REVERSE);

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        while (opModeInInit()){

            ServoGripperLeft.setPosition(0.5);
            ServoGripperRight.setPosition(0.5);

            ServoIntakeLeft.setPosition(1);
            ServoIntakeRight.setPosition(0);
        }

        waitForStart();
        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            double turn = (-gamepad1.right_stick_x * 0.5);
            double throttle = Math.pow(gamepad1.left_stick_y, 3);
            double rightspeed = throttle - turn;
            double leftspeed = throttle + turn;

            rightRear.setPower(rightspeed);
            leftRear.setPower(leftspeed);

            if(currentGamepad1.a && !previousGamepad1.a) {
                gripFlag1=!gripFlag1;
            }
            if(gripFlag1) {
                ServoGripperLeft.setPosition(0.35);
            }
            else {
                ServoGripperLeft.setPosition(0.5);
            }

            if(currentGamepad1.b && !previousGamepad1.b) {
                gripFlag2=!gripFlag2;
            }
            if(gripFlag2) {
                ServoGripperRight.setPosition(0.65);
            }
            else {
                ServoGripperRight.setPosition(0.5);
            }

            if(gamepad1.x) {
                ServoIntakeLeftPos = 1;
                ServoIntakeRightPos = 1 - ServoIntakeLeftPos;
                ServoIntakeLeft.setPosition(ServoIntakeLeftPos);
                ServoIntakeRight.setPosition(ServoIntakeRightPos);
            }
            if(gamepad1.y) {
                ServoIntakeLeftPos = 0.65;
                ServoIntakeRightPos = 1 - ServoIntakeLeftPos;
                ServoIntakeLeft.setPosition(ServoIntakeLeftPos);
                ServoIntakeRight.setPosition(ServoIntakeRightPos);
            }

            if(gamepad1.dpad_up){
                extendTo(pos, 0.5);
            }
            if (gamepad1.dpad_down){
                extendTo(initpos, 0.5);
            }
            telemetry.addData("Servo angle", ServoIntakeLeft.getPosition());
            telemetry.addData("Servo griper angle", ServoGripperRight);
            telemetry.addData("lift", motorLift.getCurrentPosition());
            telemetry.addData("lift current" , motorLift.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }
    }
    public void extendTo(int position, double power) {
        motorLift.setTargetPosition(position);
        motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLift.setPower(power);
    }

}
