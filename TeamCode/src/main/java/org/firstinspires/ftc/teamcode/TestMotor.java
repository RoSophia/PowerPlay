package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//@Config
//@Disabled
@TeleOp
public class TestMotor extends LinearOpMode {
    public void runOpMode() {
        DcMotor leftBack = hardwareMap.get(DcMotor.class, "Solace");
        while (opModeIsActive()) {
            leftBack.setPower(gamepad1.right_stick_y);
        }
    }
}

