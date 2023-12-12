package org.firstinspires.ftc.teamcode.extras;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(group = "MotorTest")
public class MotorTest extends LinearOpMode {
    private DcMotorEx motor;                                           //Declare DC motor
    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, "motor");  //Name your DC Motor0

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();                                                //waits till driver starts
        while (opModeIsActive()){

            if(gamepad1.x){                                            //conditional statement
                telemetry.addData("GamePad button:", "Pressed x");
                motor.setPower(1);                                     //Run Motor
            }
            if (gamepad1.y){

                motor.setTargetPosition(300);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor.setPower(0.5);

            }
            telemetry.addData("Motor Current", motor.getCurrent(CurrentUnit.AMPS));
            motor.setPower(0);                                         //Stop the motor9
            telemetry.update();
            //telemetry are print statements and simply prints on your screen
        }
    }
}