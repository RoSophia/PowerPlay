package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class OP_Mode_mk1 extends LinearOpMode {
    //sasiu
    public DcMotor leftBack;
    public DcMotor leftFront;
    public DcMotor rightBack;
    public DcMotor rightFront;
    //restu
    public DcMotor ridicareSlide;
    public Servo s1;
    public Servo s2;
    public Servo sspate;
    float s1pos, s2pos;
    boolean lastButtonY = false;
    boolean lastButtonX = false;
    int targetReading,rid=0;
    void ridicare(int target) {
        targetReading = target;
        ridicareSlide.setTargetPosition(targetReading);
        ridicareSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ridicareSlide.setPower(1f);
    }
    public void runOpMode()
    {
        leftBack = hardwareMap.get(DcMotor.class ,"LB");
        leftFront = hardwareMap.get(DcMotor.class ,"LF");
        rightBack = hardwareMap.get(DcMotor.class ,"RB");
        rightFront = hardwareMap.get(DcMotor.class ,"RF");
        ridicareSlide = hardwareMap.get(DcMotor.class ,"RS");
        s1 = hardwareMap.get(Servo.class, "S1");
        s2 = hardwareMap.get(Servo.class, "S2");
        sspate = hardwareMap.get(Servo.class, "RPS");

       // leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

       // rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

       // leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

       // rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ridicareSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // ridicareSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ridicareSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //reverse those suckers
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

        s1pos=0.1f;
        s2pos=0.62f;
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
           // s1.setPosition(0.1);//start(si s1 spre 0.5
           // s2.setPosition(0.62);//satrt(s2 se steange spre 1
            ridicareSlide.setPower(0.8*gamepad2.left_stick_y);

            telemetry.addData("Ridicare:", ridicareSlide.getCurrentPosition());
            telemetry.update();
            /*if(gamepad1.a)
                s1.setPosition();
            if(gamepad1.b) {
                s1.setPosition();
            }
            if(gamepad1.x) {
                s2.setPosition();
            }
            if (gamepad1.y) {
                s2.setPosition();
            }*/

            if(gamepad2.a) {
                while(ridicareSlide.getCurrentPosition()!=-4120)
                    ridicare(-4120);
            }

            if(gamepad2.b) {
                while(ridicareSlide.getCurrentPosition()!=0)
                ridicare(0);
            }


            if (lastButtonY != gamepad2.y && gamepad2.y) {
                s1pos = 0.0f;
                s2pos = 1f;

            }
            lastButtonY = gamepad2.y;
            if (lastButtonX != gamepad2.x && gamepad2.x) {
                s1pos = 0.1f;
                s2pos = 0.62f;
            }
            lastButtonX = gamepad2.x;



            s1.setPosition(s1pos);
            s2.setPosition(s2pos);

            //dam blana in motoare
            leftFront.setPower(lfPower);
            rightFront.setPower(rfPower);
            leftBack.setPower(lbPower);
            rightBack.setPower(rbPower);
        }
    }
}

