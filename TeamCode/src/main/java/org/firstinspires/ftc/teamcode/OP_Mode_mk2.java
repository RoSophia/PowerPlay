package org.firstinspires.ftc.teamcode;

/*
         ____    _    ____ ___    _      _ _____  ___   ___
        |  _ \  / \  / ___|_ _|  / \    / |___ / / _ \ / _ \
        | | | |/ _ \| |    | |  / _ \   | | |_ \| | | | | | |
        | |_| / ___ \ |___ | | / ___ \  | |___) | |_| | |_| |
        |____/_/   \_\____|___/_/   \_\ |_|____/ \___/ \___/

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
 * SOFT ARABESC PERVERS VERSIUNE 0.24c
 * AUTOR: VERICU
 * E PERVERS, RUPE ADERENTA
 *
 * Cu de toate fara ceapa boss
 */

import static org.firstinspires.ftc.teamcode.RobotVars.BOT_POS;
import static org.firstinspires.ftc.teamcode.RobotVars.MID_POS;
import static org.firstinspires.ftc.teamcode.RobotVars.MIU_POS;
import static org.firstinspires.ftc.teamcode.RobotVars.S1PO;
import static org.firstinspires.ftc.teamcode.RobotVars.S2PO;
import static org.firstinspires.ftc.teamcode.RobotVars.S3PO;
import static org.firstinspires.ftc.teamcode.RobotVars.SDESCHIS;
import static org.firstinspires.ftc.teamcode.RobotVars.SINCHIS;
import static org.firstinspires.ftc.teamcode.RobotVars.TOP_POS;
import static org.firstinspires.ftc.teamcode.RobotVars.USE_PHOTON;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@TeleOp
public class OP_Mode_mk2 extends LinearOpMode {
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
    public static double POW_COEF = 0.9;
    public static double COB_COEF = 0.5;
    public static boolean OV3RDR1V3 = false;

    public static int BRAK = 1;

    public static int dif = 170;
    public static double DIFP = 1.2;

    public static boolean LOV = false;
    public static double UPPS = 100;
    double UPP = 100;
    public static double UPPP = 100;

    DcMotor underglow;

    public double FC = 0.5;
    public double SPC = 1.2;
    public double MINP = 0.2;

    public static double P1 = 1;
    public static double P2 = 1;
    public static double P3 = 1;
    public static double P4 = 1;
    double luv = 0;
    public static double CSM = -0.00015;

    ElapsedTime et = new ElapsedTime(0);

    void upd_underglow(double speed) { /// Underglow cam pervers
        double cs = Math.abs(speed) * Math.sqrt(2) / 2 * SPC;
        if (cs > luv) {
            luv = cs;
        } else {
            luv -= et.seconds() * FC;
        }
        luv = Math.min(Math.max(luv, MINP), 1);

        underglow.setPower(-luv);
        et.reset();
    }

    public double WT = 0.2;
    public static double HMIN = 0.005;

    public double MDIST = 48;
    public double MMDIST = 25;
    DistanceSensor ss, sd, cs;

    public void runOpMode() {
        if (USE_PHOTON) {
            PhotonCore.enable();
            PhotonCore.experimental.setSinglethreadedOptimized(false);
        }
        VoltageSensor batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        FtcDashboard dashboard = FtcDashboard.getInstance();

        L2A = L2B = L2U = L2D = G2X = R2RB = R2LB = R2LT = RB = switched = OV3RDR1V3 = false;
        UPP = UPPS;

        leftBack = hardwareMap.get(DcMotorEx.class, "LB");
        leftFront = hardwareMap.get(DcMotorEx.class, "LF");
        rightBack = hardwareMap.get(DcMotorEx.class, "RB");
        rightFront = hardwareMap.get(DcMotorEx.class, "RF");
        ridicareSlide = hardwareMap.get(DcMotorEx.class, "RS");
        underglow = hardwareMap.get(DcMotor.class, "Underglow");

        cs = hardwareMap.get(DistanceSensor.class, "Claw");
        /*sd = hardwareMap.get(Rev2mDistanceSensor.class, "SD");
        ss = hardwareMap.get(Rev2mDistanceSensor.class, "SS");*/

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        /*parameters.mode = BNO055IMU.SensorMode.COMPASS;
        parameters.gyroPowerMode = BNO055IMU.GyroPowerMode.FAST;
        parameters.gyroBandwidth = BNO055IMU.GyroBandwidth.HZ523; /// TODO ???????
        parameters.gyroRange = BNO055IMU.GyroRange.DPS2000;*/
        imu.initialize(parameters);

        Runnable armRun = new ArmcPIDF(ridicareSlide);
        Thread armThread = new Thread(armRun);

        s1 = hardwareMap.get(Servo.class, "S1");
        //s2 = hardwareMap.get(Servo.class, "S2"); De ce era asta aici?
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

        s1.setPosition(SDESCHIS);
        S1.setPosition(S1PO);
        S2.setPosition(S2PO);
        S3.setPosition(S3PO);

        waitForStart();

        ThreadInfo.shouldClose = false;
        ThreadInfo.pcoef = 12.0 / batteryVoltageSensor.getVoltage();
        ThreadInfo.use = true;
        ThreadInfo.target = 0;
        armThread.start();

        ElapsedTime timer = new ElapsedTime(0);
        ElapsedTime g1t = new ElapsedTime(0);
        double lhpos = 0;
        while (opModeIsActive()) {
            S1.setPosition(S1PO);
            S2.setPosition(S2PO);
            S3.setPosition(S3PO);

            if (ThreadInfo.useTele) {
                TelemetryPacket fp = new TelemetryPacket();
                fp.put("CycleTime", timer.milliseconds());
                timer.reset();
                dashboard.sendTelemetryPacket(fp);
            }

            final double speed = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double ch = imu.getAngularOrientation().firstAngle;
            final double angle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;// + ch;
            double hdif = 0;
            if (Math.abs(gamepad1.right_stick_x) > 0.01) {
                g1t.reset();
            }
            if (g1t.seconds() < WT) {
                lhpos = ch;
            } else {
                hdif = lhpos - ch;
                if (hdif > Math.PI / 2) {
                    hdif = Math.PI - hdif;
                } else if (hdif < -Math.PI / 2) {
                    hdif -= Math.PI;
                }
                if (Math.abs(hdif) < HMIN || speed > 0.01) {
                    hdif = 0;
                }
            }
            TelemetryPacket pack = new TelemetryPacket();
            //pack.put("lhpos", lhpos);
            pack.put("ch", ch);
            pack.put("Acc", imu.getCalibrationStatus());
            pack.put("cs", cs.getDistance(DistanceUnit.MM));
            //pack.put("dif", hdif);
            dashboard.sendTelemetryPacket(pack);
            final double turn = hdif * DIFP - gamepad1.right_stick_x;
            final double ms = speed * Math.sin(angle);
            final double mc = speed * Math.cos(angle);
            //maths (nu stiu eu deastea ca fac cu antohe)
            final double lfPower = ms + turn;
            final double rfPower = mc - turn;
            final double lbPower = mc + turn;
            final double rbPower = ms - turn;
            upd_underglow(Math.abs(turn) + Math.abs(speed));

            if (!L2A && gamepad2.a) {
                ThreadInfo.target = TOP_POS + (int) (gamepad2.left_trigger * UPP);
            }
            L2A = gamepad2.a;

            if (!L2RB && gamepad2.right_bumper) {
                UPP += UPPP;
            }
            L2RB = gamepad2.right_bumper;

            if (!L2B && gamepad2.b) {
                ThreadInfo.target = BOT_POS + (int) (gamepad2.left_trigger * UPP);
            }
            L2B = gamepad2.b;

            if (!L2U && gamepad2.dpad_up) {
                ThreadInfo.target = MIU_POS + (int) (gamepad2.left_trigger * UPP);
            }
            L2U = gamepad2.dpad_up;

            if (!L2D && gamepad2.dpad_down) {
                ThreadInfo.target = MID_POS + (int) (gamepad2.left_trigger * UPP);
            }
            L2D = gamepad2.dpad_down;

            if (!R2RB && gamepad2.dpad_left &&
                    !R2LB && gamepad2.left_bumper
            ) {
                OV3RDR1V3 = !OV3RDR1V3;
                telemetry.addLine("Nothing here, keep looking");
                telemetry.update();
            }
            R2RB = gamepad2.right_bumper;
            R2LB = gamepad2.left_bumper;

            if (USE_TELEMETRY) {
                telemetry.clear();
                telemetry.clearAll();
                telemetry.addLine("PT");
            }
            final double DPC = 1 - 0.6 * gamepad2.right_trigger;
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
                    ridicareSlide.setPower(POW_COEF * -gamepad2.right_stick_y * DPC);
                } else {
                    ThreadInfo.use = true;
                    if (LOV) {
                        ridicareSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                        ridicareSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                        LOV = false;
                    }
                    if (USE_TELEMETRY) {
                        telemetry.addLine("0V3RDR1V3 AUT");
                    }

                    ThreadInfo.target = 0;
                }
            } else {
                if (Math.abs(gamepad2.right_stick_y) > 0.01) {
                    ThreadInfo.use = false;
                    if (USE_TELEMETRY) {
                        telemetry.addLine("ALL MAN");
                    }
                    lpos = ridicareSlide.getCurrentPosition();
                    ThreadInfo.fast = true;
                    if (lpos < 0) {
                        if (USE_TELEMETRY) {
                            telemetry.addLine("MAN UND");
                        }
                        ThreadInfo.target = 10;
                    } else if (lpos < 150 && gamepad2.right_stick_y > 0) {
                        if (USE_TELEMETRY) {
                            telemetry.addLine("MAN CLOSE UND");
                        }
                        ridicareSlide.setPower(0);
                        ThreadInfo.target = lpos;
                    } else if (lpos > TOP_POS - 40 && gamepad2.right_stick_y < 0) {
                        if (USE_TELEMETRY) {
                            telemetry.addLine("MAN CLOSE UPW");
                        }
                        ridicareSlide.setPower(0);
                        ThreadInfo.target = lpos;
                    } else if (lpos < TOP_POS) {
                        if (gamepad2.right_stick_y > 0) {
                            if (USE_TELEMETRY) {
                                telemetry.addLine("COB MAN");
                            }
                            ridicareSlide.setPower(COB_COEF * -gamepad2.right_stick_y * DPC);
                        } else {
                            if (USE_TELEMETRY) {
                                telemetry.addLine("RID MAN");
                            }
                            ridicareSlide.setPower(POW_COEF * -gamepad2.right_stick_y * DPC);
                        }
                        ThreadInfo.target = lpos;
                    } else {
                        if (USE_TELEMETRY) {
                            telemetry.addLine("MAN UPW");
                        }
                        ridicareSlide.setPower(AUT_POW);
                        ThreadInfo.target = TOP_POS;
                    }
                } else {
                    ThreadInfo.use = true;
                    ThreadInfo.fast = false;
                    if (USE_TELEMETRY) {
                        telemetry.addLine("ALL AUT");
                    }
                    if (ThreadInfo.target < ridicareSlide.getCurrentPosition()) {
                        if (USE_TELEMETRY) {
                            telemetry.addLine("COB AUT");
                        }
                    } else {
                        if (USE_TELEMETRY) {
                            telemetry.addLine("RID AUT");
                        }
                    }
                }
            }
            if (USE_TELEMETRY) {
                telemetry.addLine("TEST");
            }

            if (!RB && gamepad2.left_bumper && !gamepad2.right_bumper) {
                ThreadInfo.fast = true;
                ThreadInfo.target = ThreadInfo.target - dif;
            }
            if (RB && !gamepad2.left_bumper && !gamepad2.right_bumper) {
                ThreadInfo.fast = false;
                ThreadInfo.target = ThreadInfo.target + dif;
            }
            RB = gamepad2.left_bumper;

            if (USE_TELEMETRY) {
                if (OV3RDR1V3) {
                    telemetry.addLine("U HAV3 3NT3R3D 0V3RDR1V3 M0D3");
                }
                telemetry.addData("Ridicare", ridicareSlide.getCurrentPosition());
                telemetry.addData("Target", ThreadInfo.target);
                telemetry.addData("Cpow", ridicareSlide.getPower());
                telemetry.addData("left_stick_y", poz);
                telemetry.addData("r_stick_y", gamepad2.right_stick_y);
                telemetry.addData("pad", gamepad1.right_trigger);
                telemetry.addData("PID Fps", ThreadInfo.fr);
                telemetry.update();
            } else if (OV3RDR1V3) {
                telemetry.addLine("U HAV3 3NT3R3D 0V3RDR1V3 M0D3");
                telemetry.update();
            }

            final double cd = cs.getDistance(DistanceUnit.MM);
            if (!G2X && gamepad2.x) {
                if (switched) {
                    s1.setPosition(SDESCHIS);
                    switched = false;
                } else {
                    if (cd < MDIST) {
                        if (cd < MMDIST) {
                            s1.setPosition(SINCHIS);
                        } else {
                            s1.setPosition(SINCHIS + (MDIST - cs.getDistance(DistanceUnit.MM)) * CSM);
                        }
                        switched = true;
                    }
                }
            }
            G2X = gamepad2.x;
            /*if (cd < MDIST) {
                s1.setPosition(SINCHIS + (MDIST - cs.getDistance(DistanceUnit.MM)) * CSM);
                switched = true;
            } else {
                s1.setPosition(SDESCHIS);
            }*/

            //dam blana in motoare
            ThreadInfo.pcoef = 12.0 / batteryVoltageSensor.getVoltage();
            final double spcoef = 1 - 0.6 * gamepad1.right_trigger;
            final double fcoef = ThreadInfo.pcoef * spcoef;
            leftFront.setPower(lfPower * fcoef * P1);  // LB
            rightFront.setPower(rfPower * fcoef * P2); // LF
            leftBack.setPower(lbPower * fcoef * P3);   // RB
            rightBack.setPower(rbPower * fcoef * P4);  // RF
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        try {
            ThreadInfo.shouldClose = true;
            armThread.join();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
