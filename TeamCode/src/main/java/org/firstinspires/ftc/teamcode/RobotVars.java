package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotVars {
    public static double SMDESCHIS = 0.80; // Claw Very Open (After put)
    public static double SDESCHIS = 0.80; // Claw Open
    public static double SINCHIS = 0.97;   // Claw Close

    public static int EMIN = 7;          // Minimum Extension
    public static int EMAX = 630;         // Maximum Extension

    public static int RTOP_POS = 590;     // Maximum Lift Position
    public static int RMIU_POS = 345;     // Medium Lift Position
    public static int RBOT_POS = 0;       // Bottom Lift position
    public static int LEEW = 0;           // Leeway For Manual Lift Movement

    public static double SAG = 0.420;     // Grabber Arm Get position
    public static double SAH = 0.610;     // Grabber Arm Hold Position
    public static double SAP = 0.77;      // Grabber Arm Put position
    public static double SAW = 0.680;     // Grabber Arm Wait position

    public static double SBAG = 0.59;     // 4bar Get position
    public static double SBAH = 0.39;     // 4bar Hold position
    public static double SBAC = 0.57;     // 4bar Hold position
    public static double SBAP = 0.77;     // 4bar Put position
    public static double SBAAP = 0.9;     // 4bar Put position

    public static double SDIF = 0.076;      // Grabber Arm Servo Diff Term
    public static double SDIP = 0.0;      // Grabber Arm Servo Proportional Drift Term

    public static double SHG = 0.09;     // Grabber Get Heading
    public static double SHP = 0.09;      // Grabber Put Heading
    public static double SHAP = 0.15;     // Grabber After Put Heading

    public static double SCO = 0.0;       // Cone Securing Mini Servo Open Position
    public static double SCM = 0.6;       // Cone Securing Mini Servo Medium Position
    public static double SCC = 0.86;       // Cone Securing Mini Servo Close Position

    public static double SGS = 0.505;  // Cone stack start position
    public static double SGD = 0.021;  // Cone stack diff
    public static double SBAS = 0.60;  // Cone stack balance start position
    public static double SBAD = 0.0;   // Cone stack balance diff

    public static double DOT = 0.0;       // Time To Lower The Lift
    public static double UPT = 0.5;       // Time To Hoist Up The Thing
    public static double MUPT = 0.3;       // Time To Hoist Up The Thing
    public static double EXTT = 0.8;      // Time To Fully Extend
    public static double RETT = 0.0;      // Time To Retract

    public static boolean USE_PHOTON = true;

    public static boolean USE_TELE = true; // Use Telemetry
    public static boolean USE_TELE_MOVE = true; // Use Telemetry
    public static int CU_TESTING = 0;
    public static double pcoef = 1.0; // Voltage Normalising Term (set during runtime)

    public static boolean coneClaw = false;
    public static boolean armHolding = false;
    public static boolean coneReady = false;

    public static double ep = 0.0023;  // Extension pidf and correction term between the motors
    public static double ed = 0.00001;
    public static double ei = 0.0;
    public static double ef = 0;
    public static double emd = 10;
    public static double ebp = 0.0000;

    public static double rp = 0.005;  // Lift pidf and correction term between the motors
    public static double rd = 0.0;
    public static double ri = 0.0;
    public static double rf = 0.0006;
    public static double rmd = 15;
    public static double rbp = 0.018;

    public static double EAP = 1.0; // Extension A Scale Term
    public static double EBP = 1.0; // Extension B Scale Term
    public static double RAP = 1.0; // Lift A Scale Term
    public static double RBP = 1.0;

    public static boolean useExt = true;
    public static boolean useRid = true;

    public static String LES = "RF";
    public static String RES = "RB";
    public static String FES = "LB";
    public static boolean LER = true;
    public static boolean RER = true;
    public static boolean FER = true;

    public static boolean USE_UPD_HEAD = true;
    public static boolean USE_UPD_HEAD_FULL = false;

    public static String RIDICARE_LAMPREY = "lamprey2";

    public static boolean AUTO_CLOW = false;

    public static double SHITTY_WORKAROUND_TIME = 0.2;
    public static double SHITTY_WORKAROUND_POWER = 1;

    public static int MAX_DIF_RID = 5;
    public static int MAX_DIF_EXT = 25;

    public static int LOCALIZATION_SLEEP_TIME = 5;

     /*
     * Expansion:
     *     Motors:
     *         0: LF
     *         1: RF
     *         2: RB
     *         3: LB
     *     Servos:
     *         0: sHeading
     *         1:
     *         2: sBalans
     *         3:
     *         4: sClose
     *         5:
     *      Analog:
     *         0-1: Lamprey
     * Control:
     *     Motors:
     *         0: extB
     *         1: ridA
     *         2: extA
     *         3: ridB
     *     Servos:
     *         0: Toate
     *         1: sextB
     *         2:
     *         3: sMCLaw
     *         4:
     *         5: sextA
     * Colours:
     *     Verde Cacaniu:                  sBalans  - Spm 1
     *     Verde Deschis Adi:              sHeading - Expansion 0
     *     Rosu:                           sClose   - Expansion 4
     *
     *     Normal Steag Germania -> Cablu pictat albastru: sExtA    - Spm 2
     *     Normal Steag Germania Imperial                : sMCLaw   - Spm 6
     *     Albastru:                                       sExtB    - Spm 5
     *
     *     Cacaniu: Through-bore balans
     */
}
