package org.firstinspires.ftc.teamcode.extras;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(group = "MotorTest")
public class MotorTest extends LinearOpMode {
    private DcMotorEx motor;                                           //Declare DC motor
    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, "motor");  //Name your DC Motor
        waitForStart();                                                //waits till driver starts
        while (opModeIsActive()){

            if(gamepad1.x){                                            //conditional statement
                telemetry.addData("GamePad button:", "Pressed x");
                motor.setPower(1);                                     //Run Motor
            }
            telemetry.addData("Motor Current", motor.getCurrent(CurrentUnit.AMPS));
            motor.setPower(0);                                         //Stop the motor
            telemetry.update();
            //telemetry are print statements and simply prints on your screen
        }
    }
}