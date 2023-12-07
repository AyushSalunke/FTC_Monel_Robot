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

//@Config
@TeleOp(group = "SonicMax")
public class Robot1 extends LinearOpMode {
    DcMotorEx armMotor;
    DcMotorEx leftRear, rightRear;
    Servo Servogripper;
    DcMotorEx intakeMotor;
    boolean gripFlag = false;
    public int pos = 100;
    public int initpos = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        Servogripper = hardwareMap.servo.get("Servogripper");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");

        armMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        while (opModeInInit()){
            Servogripper.setPosition(0.5);
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
                gripFlag=!gripFlag;
            }
            if(gripFlag) {
                Servogripper.setPosition(0.2);
            }
            else {
                Servogripper.setPosition(0.5);
            }

            if(gamepad1.x) {
                intakeMotor.setTargetPosition(intakeMotor.getCurrentPosition() + 20);
                intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                intakeMotor.setPower(0.5);
            }
            if(gamepad1.y)
            {
                intakeMotor.setTargetPosition(intakeMotor.getCurrentPosition() - 20);
                intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                intakeMotor.setPower(0.5);
            }

            if(gamepad1.dpad_up){
                armMotor.setTargetPosition(armMotor.getCurrentPosition() + 25);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.5);
//                extendTo(pos, 0.5);
            }
            if (gamepad1.dpad_down){
                armMotor.setTargetPosition(armMotor.getCurrentPosition() -25);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.5);
//                extendTo(initpos, 0.5);
            }
            telemetry.addData("lift", armMotor.getCurrentPosition());
            telemetry.addData("lift current" , armMotor.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("ServoGripperPos", Servogripper.getPosition());
            telemetry.update();
        }
    }
    public void extendTo(int position, double power) {
        armMotor.setTargetPosition(position);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(power);
    }
}
