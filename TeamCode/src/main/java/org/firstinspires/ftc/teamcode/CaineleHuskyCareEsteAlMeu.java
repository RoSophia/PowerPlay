package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.HuskyLensLib.EElement;

import org.firstinspires.ftc.teamcode.HuskyLensLib.HuskyLensLib;

import java.util.Vector;

@Config
@TeleOp
public class CaineleHuskyCareEsteAlMeu extends LinearOpMode {
    //sasiu
    public DcMotor leftBack;
    public DcMotor leftFront;
    public DcMotor rightBack;
    public DcMotor rightFront;

    HuskyLensLib husk;

    public void runOpMode() {
        leftBack = hardwareMap.get(DcMotor.class, "LB");
        leftFront = hardwareMap.get(DcMotor.class, "LF");
        rightBack = hardwareMap.get(DcMotor.class, "RB");
        rightFront = hardwareMap.get(DcMotor.class, "RF");
        husk = hardwareMap.get(HuskyLensLib.class, "Husky");

        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //reverse those suckers
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            double speed = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double angle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double turn = -gamepad1.right_stick_x;
            //maths (nu stiu eu deastea ca fac cu antohe)
            final double lfPower = speed * Math.sin(angle) + turn;
            final double rfPower = (speed * Math.cos(angle) - turn);
            final double lbPower = speed * Math.cos(angle) + turn;
            final double rbPower = (speed * Math.sin(angle) - turn);

            Vector<EElement> e = husk.requestAll();
            telemetry.addData("Len:", e.size());
            for (EElement x : e) {
                telemetry.addData(" \\- ID: ", x.ID);
                telemetry.addData("  - TY: ", x.type);
                telemetry.addData("  - X0: ", x.x0);
                telemetry.addData("  - Y0: ", x.y0);
                telemetry.addData("  - X1: ", x.x1);
                telemetry.addData("  - Y1: ", x.y1);
            }
            telemetry.update();

            //dam blana in motoare
            leftFront.setPower(lfPower);
            rightFront.setPower(rfPower);
            leftBack.setPower(lbPower);
            rightBack.setPower(rbPower);
        }
    }
}
