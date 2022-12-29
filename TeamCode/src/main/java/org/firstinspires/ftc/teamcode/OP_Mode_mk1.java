package org.firstinspires.ftc.teamcode;

/*
         ____   ___  _____ _____      _    ____      _    ____  _____ ____   ____
        / ___| / _ \|  ___|_   _|    / \  |  _ \    / \  | __ )| ____/ ___| / ___|
        \___ \| | | | |_    | |     / _ \ | |_) |  / _ \ |  _ \|  _| \___ \| |
        ___) | |_| |  _|   | |    / ___ \|  _ <  / ___ \| |_) | |___ ___) | |___
        |____/ \___/|_|     |_|   /_/   \_\_| \_\/_/   \_\____/|_____|____/ \____|

        ____  _____ ______     _______ ____  ____
        |  _ \| ____|  _ \ \   / / ____|  _ \/ ___|
        | |_) |  _| | |_) \ \ / /|  _| | |_) \___ \
        |  __/| |___|  _ < \ V / | |___|  _ < ___) |
        |_|   |_____|_| \_\ \_/  |_____|_| \_\____/



         _____   ____  _____ ______     _______ ____  ____
        | ____| |  _ \| ____|  _ \ \   / / ____|  _ \/ ___|
        |  _|   | |_) |  _| | |_) \ \ / /|  _| | |_) \___ \
        | |___  |  __/| |___|  _ < \ V / | |___|  _ < ___) |
        |_____| |_|   |_____|_| \_\ \_/  |_____|_| \_\____/

         ____  _   _ ____  _____      _    ____  _____ ____  _____ _   _ _____  _
        |  _ \| | | |  _ \| ____|    / \  |  _ \| ____|  _ \| ____| \ | |_   _|/ \
        | |_) | | | | |_) |  _|     / _ \ | | | |  _| | |_) |  _| |  \| | | | / _ \
        |  _ <| |_| |  __/| |___   / ___ \| |_| | |___|  _ <| |___| |\  | | |/ ___ \
        |_| \_\\___/|_|   |_____| /_/   \_\____/|_____|_| \_\_____|_| \_| |_/_/   \_\
 */

/*
 * SOFT ARABESC PERVERS VERSIUNE 0.23b
 * AUTOR: VERICU
 * E PERVERS, RUPE ADERENTA
 *
 * Cu de toate fara ceapa boss
 */

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Config
@TeleOp
public class OP_Mode_mk1 extends LinearOpMode {
    //sasiu
    public DcMotorEx leftBack;
    public DcMotorEx leftFront;
    public DcMotorEx rightBack;
    public DcMotorEx rightFront;
    //restu
    public DcMotorEx ridicareSlide;
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

    public static boolean USE_TELEMETRY = false;

    int lpos = 0;

    void ridicare(int target) {
        targetReading = target;
        ridicareSlide.setTargetPosition(targetReading);
        ridicareSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        ridicareSlide.setPower(1f);
        if (gamepad2.y) {
            s1pos = 0.0f;
            //s2pos = 0.2f;
        }
        //lastButtonY = gamepad2.y;
        if (gamepad2.x) {
            s1pos = 1f;
            //s2pos = 0.4f;
        }
    }
    //

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
    public static double POW_COEF = -0.9;
    public static double COB_COEF = 0.5;
    public static boolean OV3RDR1V3 = false;

    public static int BRAK = 1;

    public static int dif = 150;

    public static int TOP_POS = 1303;
    public static int MIU_POS = 1010;
    public static int MID_POS = 760;

    public static double SDESCHIS = 0.75;
    public static double SINCHIS = 0.25;
    //PIDController controller;
    //double p = 0, i = 0, d = 0;
    //double pep = 0;
    //double pap = 0;

    public void runOpMode() {
        VoltageSensor batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        L2A = L2B = L2U = L2D = G2X = R2RB = R2LT = RB = switched = OV3RDR1V3 = false;

        leftBack = hardwareMap.get(DcMotorEx.class, "LB");
        leftFront = hardwareMap.get(DcMotorEx.class, "LF");
        rightBack = hardwareMap.get(DcMotorEx.class, "RB");
        rightFront = hardwareMap.get(DcMotorEx.class, "RF");
        ridicareSlide = hardwareMap.get(DcMotorEx.class, "RS");

        s1 = hardwareMap.get(Servo.class, "S1");
        //s2 = hardwareMap.get(Servo.class, "S2");
        leftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        if (BRAK == 1) {
            rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }

        ridicareSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        // ridicareSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        ridicareSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        ridicareSlide.setDirection(DcMotorEx.Direction.REVERSE);
        ridicareSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //reverse those suckers
        leftBack.setDirection(DcMotorEx.Direction.FORWARD);
        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightBack.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);

        Servo SPa = hardwareMap.get(Servo.class, "SPa");
        Servo SPe = hardwareMap.get(Servo.class, "SPe");

        SPa.setPosition(0.0);
        SPe.setPosition(0.0);

        s1pos = 0.75f;
        ridicareSlide.setTargetPosition(0);

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

            if (!R2RB && gamepad2.right_bumper) {
                OV3RDR1V3 = !OV3RDR1V3;
                telemetry.addLine("Nothing here, keep looking");
                telemetry.update();
            }
            R2RB = gamepad2.right_bumper;

            if (!R2LT && gamepad2.left_trigger > 0.01) {
                USE_TELEMETRY = !USE_TELEMETRY;
                telemetry.addLine("Nothing here, keep looking");
                telemetry.update();
            }
            R2LT = gamepad2.left_trigger > 0.01;

            if (USE_TELEMETRY) {
                telemetry.clear();
                telemetry.clearAll();
                telemetry.addLine("PT");
                //drive.updatePoseEstimate();
                //telemetry.addData("Cpos", drive.getPoseEstimate());
            }
            if (OV3RDR1V3) {
                if (USE_TELEMETRY) {
                    telemetry.addLine("ALL 0V3");
                }
                if (Math.abs(gamepad2.right_stick_y) > 0.01) {
                    if (ridicareSlide.getMode() != DcMotorEx.RunMode.RUN_USING_ENCODER) {
                        ridicareSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                        ridicareSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                    }
                    if (USE_TELEMETRY) {
                        telemetry.addLine("0V3RDR1V3 MAN");
                    }
                    ridicareSlide.setPower(POW_COEF * gamepad2.right_stick_y);
                } else {
                    if (ridicareSlide.getMode() != DcMotorEx.RunMode.RUN_TO_POSITION) {
                        ridicareSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    }
                    if (USE_TELEMETRY) {
                        telemetry.addLine("0V3RDR1V3 AUT");
                    }

                    ridicareSlide.setTargetPosition(0);
                    ridicareSlide.setPower(POW_COEF / 2);
                }
            } else {
                if (Math.abs(gamepad2.right_stick_y) > 0.01) {
                    if (USE_TELEMETRY) {
                        telemetry.addLine("ALL MAN");
                    }
                    if (ridicareSlide.getMode() != DcMotorEx.RunMode.RUN_USING_ENCODER) {
                        ridicareSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                    }
                    lpos = ridicareSlide.getCurrentPosition();
                    if (lpos < 0) {
                        if (USE_TELEMETRY) {
                            telemetry.addLine("MAN UND");
                        }
                        ridicareSlide.setPower(AUT_POW);
                        ridicareSlide.setTargetPosition(10);
                    } else if (lpos < 150 && gamepad2.right_stick_y > 0) {
                        if (USE_TELEMETRY) {
                            telemetry.addLine("MAN CLOSE UND");
                        }
                        ridicareSlide.setPower(0);
                        ridicareSlide.setTargetPosition(lpos);
                    } else if (lpos > TOP_POS - 40 && gamepad2.right_stick_y < 0) {
                        if (USE_TELEMETRY) {
                            telemetry.addLine("MAN CLOSE UPW");
                        }
                        ridicareSlide.setPower(0);
                        ridicareSlide.setTargetPosition(lpos);
                    } else if (lpos < TOP_POS) {
                        if (gamepad2.right_stick_y > 0) {
                            if (USE_TELEMETRY) {
                                telemetry.addLine("COB MAN");
                            }
                            ridicareSlide.setPower(-COB_COEF * 1.3);
                        } else {
                            if (USE_TELEMETRY) {
                                telemetry.addLine("RID MAN");
                            }
                            ridicareSlide.setPower(POW_COEF * gamepad2.right_stick_y);
                        }
                        ridicareSlide.setTargetPosition(lpos);
                    } else {
                        if (USE_TELEMETRY) {
                            telemetry.addLine("MAN UPW");
                        }
                        ridicareSlide.setPower(AUT_POW);
                        ridicareSlide.setTargetPosition(TOP_POS);
                    }
                } else {
                    if (ridicareSlide.getMode() != DcMotorEx.RunMode.RUN_TO_POSITION) {
                        ridicareSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    }
                    if (USE_TELEMETRY) {
                        telemetry.addLine("ALL AUT");
                    }
                    if (ridicareSlide.getTargetPosition() < ridicareSlide.getCurrentPosition()) {
                        if (USE_TELEMETRY) {
                            telemetry.addLine("COB AUT");
                        }
                        ridicareSlide.setPower(COB_COEF);
                    } else {
                        if (USE_TELEMETRY) {
                            telemetry.addLine("RID AUT");
                        }
                        ridicareSlide.setPower(AUT_POW);
                    }
                }
            }
            if (USE_TELEMETRY) {
                telemetry.addLine("TEST");
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


            if (USE_TELEMETRY) {
                if (OV3RDR1V3) {
                    telemetry.addLine("U HAV3 3NT3R3D 0V3RDR1V3 M0D3");
                }
                telemetry.addData("Ridicare", ridicareSlide.getCurrentPosition());
                telemetry.addData("Target", ridicareSlide.getTargetPosition());
                telemetry.addData("Cpow", ridicareSlide.getPower());
                telemetry.addData("left_stick_y", poz);
                telemetry.addData("r_stick_y", gamepad2.right_stick_y);
                telemetry.addData("pad", gamepad1.right_trigger);
                telemetry.update();
            } else if (OV3RDR1V3) {
                telemetry.addLine("U HAV3 3NT3R3D 0V3RDR1V3 M0D3");
                telemetry.update();
            }

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



            /*if (Math.abs(gamepad1.left_stick_x) +
                Math.abs(gamepad1.left_stick_y) +
                Math.abs(gamepad1.right_stick_x) +
                Math.abs(gamepad1.right_stick_y) > 0.0001) {





            } else {*/
            //dam blana in motoare
            double pcoef = 12.0 / batteryVoltageSensor.getVoltage();
            double spcoef = 1 - 0.6 * gamepad1.right_trigger;
            double fcoef = pcoef * spcoef;
             leftFront.setPower(lfPower * fcoef);
            rightFront.setPower(rfPower * fcoef);
              leftBack.setPower(lbPower * fcoef);
             rightBack.setPower(rbPower * fcoef);
            //pep = SPe.getPosition();
            //pap = SPa.getPosition();
            //}
        }
    }
}

