package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Hanger {
    public static DcMotorEx hangerMotor;
    public static Servo hangerServo;
    public static double motorPower = 0.8;
    public static double extendPosition = 0.0;
    public Hanger(HardwareMap hardwareMap, Telemetry telemetry) {

        hangerServo = hardwareMap.get(Servo.class, "hangerServo");

        hangerMotor = hardwareMap.get(DcMotorEx.class, "hangerMotor");
        hangerMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        hangerMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        hangerMotor.setDirection(DcMotorEx.Direction.FORWARD);
        hangerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public static void ExtendHanger(){
        hangerServo.setPosition(extendPosition);
    }
    public static void LiftRobot(){
        hangerMotor.setTargetPosition(hangerMotor.getCurrentPosition() + 1000 );
        hangerMotor.setPower(motorPower);
        hangerMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }
    public static void PutDownRobot(){
        hangerMotor.setTargetPosition(hangerMotor.getCurrentPosition() - 1000 );
        hangerMotor.setPower(motorPower);
        hangerMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }
//    public static void StopHanger(){
//        hangerMotor.setPower(0);
//    }
}
