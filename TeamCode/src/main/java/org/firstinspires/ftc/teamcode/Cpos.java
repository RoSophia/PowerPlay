package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@TeleOp
public class Cpos extends LinearOpMode {
    //sasiu
    public DcMotor leftBack;
    public DcMotor leftFront;
    public DcMotor rightBack;
    public DcMotor rightFront;
    //restu
    public DcMotor ridicareSlide;
    public Servo s1;
    public Servo s2;
    public static double S1P = 0.24, S2P = 0.4; // Fucking you static
    //S2 0.4START 0.
    // 2 FINISH S1 0.24 START 0.5 FINISH
    public static double s1pos;
    int poz = 0, stick;
    boolean lastButtonY = false;
    boolean lastButtonX = false;
    int targetReading, rid = 0;

    int lpos = 0;
    boolean L2A = false;
    boolean L2B = false;
    boolean L2U = false;
    boolean L2D = false;
    boolean G2X = false;
    boolean R2RB = false;
    boolean R2LT = false;
    boolean RB = false;
    boolean switched = false;

    public static double AUT_POW = 0.811111212;
    public static double POW_COEF = -0.7;
    public static double COB_COEF = 0.3;
    public static boolean OV3RDR1V3 = false;

    public static int dif = 130;

    public static double SDESCHIS = 0.75;
    public static double SINCHIS = 0.25;

    public int TOP_POS = 1353;
    public int MIU_POS = 1060;
    public int MID_POS = 810;
    /*
        150.00,   3.37,  37.55
        131.47, -47.14, 270
        407 352
     */

    public void runOpMode() {
        VoltageSensor batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        L2A = L2B = L2U = L2D = G2X = R2RB = R2LT = RB = switched = OV3RDR1V3 = false;

        leftBack = hardwareMap.get(DcMotor.class, "LB");
        leftFront = hardwareMap.get(DcMotor.class, "LF");
        rightBack = hardwareMap.get(DcMotor.class, "RB");
        rightFront = hardwareMap.get(DcMotor.class, "RF");
        ridicareSlide = hardwareMap.get(DcMotor.class, "RS");

        s1 = hardwareMap.get(Servo.class, "S1");
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ridicareSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ridicareSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ridicareSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        ridicareSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //reverse those suckers
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

        s1pos = 0.75f;
        ridicareSlide.setTargetPosition(0);

        waitForStart();

        while (opModeIsActive()) {
            if (!G2X && gamepad2.x) {
                if (switched) {
                    s1pos = SDESCHIS;
                } else {
                    s1pos = SINCHIS;
                }
                switched = !switched;
            }
            G2X = gamepad2.x;
            s1.setPosition(s1pos);


            continue;
            /*
            double speed = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double angle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double turn = -gamepad1.right_stick_x;
            //maths (nu stiu eu deastea ca fac cu antohe)
            final double lfPower = speed * Math.sin(angle) + turn;
            final double rfPower = (speed * Math.cos(angle) - turn);
            final double lbPower = speed * Math.cos(angle) + turn;
            final double rbPower = (speed * Math.sin(angle) - turn);
            //stick = (int) gamepad2.left_stick_y;
            //poz=poz- stick;

            if (!L2A && gamepad2.a) {
                ridicareSlide.setTargetPosition(TOP_POS);
            }
            L2A = gamepad2.a;

            if (!L2B && gamepad2.b) {
                ridicareSlide.setTargetPosition(0);
            }
            L2B = gamepad2.b;

            if (!L2U && gamepad2.dpad_up) {
                ridicareSlide.setTargetPosition(MIU_POS);
            }
            L2U = gamepad2.dpad_up;

            if (!L2D && gamepad2.dpad_down) {
                ridicareSlide.setTargetPosition(MID_POS);
            }
            L2D = gamepad2.dpad_down;

            telemetry.clear();
            telemetry.clearAll();
            telemetry.addLine("PT");
            drive.updatePoseEstimate();
            telemetry.addData("Cpos", drive.getPoseEstimate());
            if (Math.abs(gamepad2.right_stick_y) > 0.01) {
                if (ridicareSlide.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                    ridicareSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                lpos = ridicareSlide.getCurrentPosition();
                if (lpos < 0) {
                    ridicareSlide.setPower(AUT_POW);
                    ridicareSlide.setTargetPosition(10);
                } else if (lpos < 150 && gamepad2.right_stick_y > 0) {
                    ridicareSlide.setPower(0);
                    ridicareSlide.setTargetPosition(lpos);
                } else if (lpos > TOP_POS - 40 && gamepad2.right_stick_y < 0) {
                    ridicareSlide.setPower(0);
                    ridicareSlide.setTargetPosition(lpos);
                } else if (lpos < TOP_POS) {
                    if (gamepad2.right_stick_y > 0) {
                        ridicareSlide.setPower(-COB_COEF * 1.3);
                    } else {
                        ridicareSlide.setPower(POW_COEF * gamepad2.right_stick_y);
                    }
                    ridicareSlide.setTargetPosition(lpos);
                } else {
                    ridicareSlide.setPower(AUT_POW);
                    ridicareSlide.setTargetPosition(TOP_POS);
                }
            } else {
                if (ridicareSlide.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                    ridicareSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                if (ridicareSlide.getTargetPosition() < ridicareSlide.getCurrentPosition()) {
                    ridicareSlide.setPower(COB_COEF);
                } else {
                    ridicareSlide.setPower(AUT_POW);
                }
            }
            if (!RB && gamepad2.left_bumper) {
                ridicareSlide.setTargetPosition(ridicareSlide.getCurrentPosition() - dif);
                ridicareSlide.setPower(1.0);
            }
            if (RB && !gamepad2.left_bumper) {
                ridicareSlide.setTargetPosition(ridicareSlide.getCurrentPosition() + dif);
                ridicareSlide.setPower(1.0);
            }
            RB = gamepad2.left_bumper;
            // ridicare(poz);


            telemetry.addData("Ridicare", ridicareSlide.getCurrentPosition());
            telemetry.addData("Target", ridicareSlide.getTargetPosition());
            telemetry.addData("Cpow", ridicareSlide.getPower());
            telemetry.addData("left_stick_y", poz);
            telemetry.addData("r_stick_y", gamepad2.right_stick_y);
            telemetry.update();

            /*
            -10.128 -2.419 30.35 grade


            if (!G2X && gamepad2.x) {
                if (switched) {
                    s1pos = 0.75f;
                } else {
                    s1pos = 0.25f;
                }
                switched = !switched;
            }
            G2X = gamepad2.x;


            s1.setPosition(s1pos);
            //s2.setPosition(s2pos);

            //dam blana in motoare
            double pcoef = 12.0 / batteryVoltageSensor.getVoltage();
            double spcoef = 1 - 0.6 * gamepad1.right_trigger;
            double fcoef = pcoef * spcoef;
            leftFront.setPower(lfPower * fcoef);
            rightFront.setPower(rfPower * fcoef);
            leftBack.setPower(lbPower * fcoef);
            rightBack.setPower(rbPower * fcoef);*/
        }
    }
}

