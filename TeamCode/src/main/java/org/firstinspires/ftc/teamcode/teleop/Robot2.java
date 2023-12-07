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
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(group = "RobotMasters")
public class Robot2 extends LinearOpMode {
    double PULLEY_DIAMETER = 30.00; //mm
    double CIRCUMFERENCE = PULLEY_DIAMETER * Math.PI;
    double GEAR_RATIO = 18.90;
    double COUNT_PER_REV = GEAR_RATIO * 28;
    double REV_PER_COUNT = 1/COUNT_PER_REV;
    double MM_PER_COUNT = REV_PER_COUNT * CIRCUMFERENCE;
    double COUNT_PER_MM = 1/ MM_PER_COUNT;

    DcMotorEx motorLift;
    DcMotorEx leftRear;
    DcMotorEx rightRear;
    Servo Servo1 ;
    Servo Servo2;
    boolean gripFlag = false;
    public double ServoPos1 = 0.0;
    public double ServoPos2 = 0.5;
    public int pos = 100;
    public int initpos = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        motorLift = hardwareMap.get(DcMotorEx.class, "motorLift");
        Servo1 = hardwareMap.servo.get("Servo1");//0
        Servo2 = hardwareMap.servo.get("Servo2");//0
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");

        motorLift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLift.setDirection(DcMotorSimple.Direction.REVERSE);

        while (opModeInInit()){
            Servo1.setPosition(0.5);
            Servo2.setPosition(0.5);
        }

        waitForStart();
        while (!isStopRequested()) {

            double turn = (-gamepad1.right_stick_x * 0.5);
            double throttle = Math.pow(gamepad1.left_stick_y, 3);
            double rightspeed = throttle - turn;
            double leftspeed = throttle + turn;

            rightRear.setPower(rightspeed);
            leftRear.setPower(leftspeed);

            if(gamepad1.a) {
                gripFlag=!gripFlag;
            }
            if(gripFlag) {
                Servo1.setPosition(0.5);
            }
            else {
                Servo1.setPosition(1);
            }

            if(gamepad1.x)
            {
                Servo2.setPosition(Servo2.getPosition() + 0.1); //increment for servo position
            }
            if(gamepad1.y)
            {
                Servo2.setPosition(Servo2.getPosition() - 0.1); //decrement for servo position
            }

            if(gamepad1.dpad_up){
                extendTo(pos, 0.5);
            }
            if (gamepad1.dpad_down){
                extendTo(initpos, 0.5);
            }
            telemetry.addData("Servo angle", Servo2.getPosition());
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
