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

import static org.firstinspires.ftc.teamcode.RobotConstants.MID_POS;
import static org.firstinspires.ftc.teamcode.RobotConstants.MIU_POS;
import static org.firstinspires.ftc.teamcode.RobotConstants.S1PO;
import static org.firstinspires.ftc.teamcode.RobotConstants.S2PO;
import static org.firstinspires.ftc.teamcode.RobotConstants.S3PO;
import static org.firstinspires.ftc.teamcode.RobotConstants.SDESCHIS;
import static org.firstinspires.ftc.teamcode.RobotConstants.SINCHIS;
import static org.firstinspires.ftc.teamcode.RobotConstants.TOP_POS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    int poz = 0;

    public static boolean USE_TELEMETRY = false;

    int lpos = 0;

    //

    boolean L2A = false;
    boolean L2RB = false;
    boolean L2B = false;
    boolean L2U = false;
    boolean L2D = false;
    boolean G2X = false;
    boolean R2RB = false;
    boolean R2LB = false;
    boolean R2LT = false;
    boolean RB = false;
    boolean switched = false;

    public static double AUT_POW = 0.811111212;
    public static double POW_COEF = -0.9;
    public static double COB_COEF = 0.5;
    public static boolean OV3RDR1V3 = false;

    public static int BRAK = 1;

    public static int dif = 170;

    public static boolean LOV = false;
    public static double UPP = 100;
    public static double UPPP = 100;

    public void runOpMode() {
        VoltageSensor batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        FtcDashboard dashboard = FtcDashboard.getInstance();

        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        L2A = L2B = L2U = L2D = G2X = R2RB = R2LB = R2LT = RB = switched = OV3RDR1V3 = false;

        leftBack = hardwareMap.get(DcMotorEx.class, "LB");
        leftFront = hardwareMap.get(DcMotorEx.class, "LF");
        rightBack = hardwareMap.get(DcMotorEx.class, "RB");
        rightFront = hardwareMap.get(DcMotorEx.class, "RF");
        ridicareSlide = hardwareMap.get(DcMotorEx.class, "RS");

        Runnable armRun = new ArmcPIDF(ridicareSlide);
        Thread armThread = new Thread(armRun);

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

        Servo S1 = hardwareMap.get(Servo.class, "SPe");
        Servo S2 = hardwareMap.get(Servo.class, "SPa1");
        Servo S3 = hardwareMap.get(Servo.class, "SPa2");

        S1.setPosition(S1PO);
        S2.setPosition(S2PO);
        S3.setPosition(S3PO);

        s1.setPosition(SDESCHIS);

        waitForStart();

        ThreadInfo.shouldClose = false;
        armThread.start();
        ThreadInfo.use = true;
        ThreadInfo.target = 0;

        /*TelemetryPacket pack;
        while (opModeIsActive()) {
            ThreadInfo.use = true;
        }*/

        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        int fps = 0;
        while (opModeIsActive()) {
            if (ThreadInfo.useTele) {
                ++fps;
                if (timer.seconds() >= 1.0) {
                    TelemetryPacket fp = new TelemetryPacket();
                    fp.put("fps", fps / timer.seconds());
                    dashboard.sendTelemetryPacket(fp);
                    fps = 0;
                    timer.reset();
                }
            }

            final double speed = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            final double angle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            final double turn = -gamepad1.right_stick_x;
            final double ms = speed * Math.sin(angle);
            final double mc = speed * Math.cos(angle);
            //maths (nu stiu eu deastea ca fac cu antohe)
            final double lfPower = ms + turn;
            final double rfPower = mc - turn;
            final double lbPower = mc + turn;
            final double rbPower = ms - turn;

            if (!L2A && gamepad2.a) {
                ThreadInfo.target = TOP_POS + (int) (gamepad2.left_trigger * UPP);
                //ridicareSlide.setTargetPosition(TOP_POS);
            }
            L2A = gamepad2.a;

            if (!L2RB && gamepad2.right_bumper) {
                UPP += UPPP;
            }
            L2RB = gamepad2.right_bumper;

            if (!L2B && gamepad2.b) {
                ThreadInfo.target = 15;
                //ridicareSlide.setTargetPosition(0);
            }
            L2B = gamepad2.b;

            if (!L2U && gamepad2.dpad_up) {
                ThreadInfo.target = MIU_POS + (int) (gamepad2.left_trigger * UPP);
                //ridicareSlide.setTargetPosition(MIU_POS);
            }
            L2U = gamepad2.dpad_up;

            if (!L2D && gamepad2.dpad_down) {
                ThreadInfo.target = MID_POS + (int) (gamepad2.left_trigger * UPP);
                //ridicareSlide.setTargetPosition(MID_POS);
            }
            L2D = gamepad2.dpad_down;

            if (!R2RB && gamepad2.right_bumper &&
                    !R2LB && gamepad2.left_bumper
            ) {
                OV3RDR1V3 = !OV3RDR1V3;
                telemetry.addLine("Nothing here, keep looking");
                telemetry.update();
            }
            R2RB = gamepad2.right_bumper;
            R2LB = gamepad2.left_bumper;

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
                    ThreadInfo.use = false;
                    LOV = true;
                    if (USE_TELEMETRY) {
                        telemetry.addLine("0V3RDR1V3 MAN");
                    }
                    ridicareSlide.setPower(POW_COEF * gamepad2.right_stick_y);
                } else {
                    ThreadInfo.use = true;
                    if (LOV) {
                        ridicareSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                        ridicareSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                        LOV = false;
                    }
                    /*if (ridicareSlide.getMode() != DcMotorEx.RunMode.RUN_TO_POSITION) {
                        ridicareSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    }*/
                    if (USE_TELEMETRY) {
                        telemetry.addLine("0V3RDR1V3 AUT");
                    }

                    ThreadInfo.target = 0;
                    /*ridicareSlide.setTargetPosition(0);
                    ridicareSlide.setPower(POW_COEF / 2);*/
                }
            } else {
                if (Math.abs(gamepad2.right_stick_y) > 0.01) {
                    ThreadInfo.use = false;
                    if (USE_TELEMETRY) {
                        telemetry.addLine("ALL MAN");
                    }
                    /*if (ridicareSlide.getMode() != DcMotorEx.RunMode.RUN_USING_ENCODER) {
                        ridicareSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                    }*/
                    lpos = ridicareSlide.getCurrentPosition();
                    if (lpos < 0) {
                        if (USE_TELEMETRY) {
                            telemetry.addLine("MAN UND");
                        }
                        ThreadInfo.target = 10;
                        //ridicareSlide.setPower(AUT_POW);
                        //ridicareSlide.setTargetPosition(10);
                    } else if (lpos < 150 && gamepad2.right_stick_y > 0) {
                        if (USE_TELEMETRY) {
                            telemetry.addLine("MAN CLOSE UND");
                        }
                        ridicareSlide.setPower(0);
                        ThreadInfo.target = lpos;
                        //ridicareSlide.setTargetPosition(lpos);
                    } else if (lpos > TOP_POS - 40 && gamepad2.right_stick_y < 0) {
                        if (USE_TELEMETRY) {
                            telemetry.addLine("MAN CLOSE UPW");
                        }
                        ridicareSlide.setPower(0);
                        ThreadInfo.target = lpos;
                        //ridicareSlide.setTargetPosition(lpos);
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
                        ThreadInfo.target = lpos;
                        //ridicareSlide.setTargetPosition(lpos);
                    } else {
                        if (USE_TELEMETRY) {
                            telemetry.addLine("MAN UPW");
                        }
                        ridicareSlide.setPower(AUT_POW);
                        ThreadInfo.target = TOP_POS;
                        //ridicareSlide.setTargetPosition(TOP_POS);
                    }
                } else {
                    ThreadInfo.use = true;
                    /*if (ridicareSlide.getMode() != DcMotorEx.RunMode.RUN_TO_POSITION) {
                        ridicareSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    }*/
                    if (USE_TELEMETRY) {
                        telemetry.addLine("ALL AUT");
                    }
                    //if (ridicareSlide.getTargetPosition() < ridicareSlide.getCurrentPosition()) {
                    if (ThreadInfo.target < ridicareSlide.getCurrentPosition()) {
                        if (USE_TELEMETRY) {
                            telemetry.addLine("COB AUT");
                        }
                        //ridicareSlide.setPower(COB_COEF);
                    } else {
                        if (USE_TELEMETRY) {
                            telemetry.addLine("RID AUT");
                        }
                        //ridicareSlide.setPower(AUT_POW);
                    }
                }
            }
            if (USE_TELEMETRY) {
                telemetry.addLine("TEST");
            }

            if (!RB && gamepad2.left_bumper && !gamepad2.right_bumper) {
                //ridicareSlide.setPower(1.0);
                ThreadInfo.target = ThreadInfo.target - dif;
                //ridicareSlide.setTargetPosition(ridicareSlide.getCurrentPosition() - dif);
            }
            if (RB && !gamepad2.left_bumper && !gamepad2.right_bumper) {
                //ridicareSlide.setPower(1.0);
                ThreadInfo.target = ThreadInfo.target + dif;
                //ridicareSlide.setTargetPosition(ridicareSlide.getCurrentPosition() + dif);
            }
            RB = gamepad2.left_bumper;

            if (USE_TELEMETRY) {
                if (OV3RDR1V3) {
                    telemetry.addLine("U HAV3 3NT3R3D 0V3RDR1V3 M0D3");
                }
                telemetry.addData("Ridicare", ridicareSlide.getCurrentPosition());
                telemetry.addData("Target", ThreadInfo.target);//ridicareSlide.getTargetPosition());
                telemetry.addData("Cpow", ridicareSlide.getPower());
                telemetry.addData("left_stick_y", poz);
                telemetry.addData("r_stick_y", gamepad2.right_stick_y);
                telemetry.addData("pad", gamepad1.right_trigger);
                telemetry.addData("PID Fps", ThreadInfo.fr);
                telemetry.addData("Rob Fps", fps);
                telemetry.update();
            } else if (OV3RDR1V3) {
                telemetry.addLine("U HAV3 3NT3R3D 0V3RDR1V3 M0D3");
                telemetry.update();
            }

            if (!G2X && gamepad2.x) {
                if (switched) {
                    s1.setPosition(SDESCHIS);
                } else {
                    s1.setPosition(SINCHIS);
                }
                switched = !switched;
            }
            G2X = gamepad2.x;

            //dam blana in motoare
            final double pcoef = 12.0 / batteryVoltageSensor.getVoltage();
            final double spcoef = 1 - 0.6 * gamepad1.right_trigger;
            final double fcoef = pcoef * spcoef;
            leftFront.setPower(lfPower * fcoef);
            rightFront.setPower(rfPower * fcoef);
            leftBack.setPower(lbPower * fcoef);
            rightBack.setPower(rbPower * fcoef);
        }

        try {
            ThreadInfo.shouldClose = true;
            armThread.join();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}