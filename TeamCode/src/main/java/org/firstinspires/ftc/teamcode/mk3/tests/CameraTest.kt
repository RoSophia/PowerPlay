package org.firstinspires.ftc.teamcode.mk3.tests

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.RobotVars.*
import org.firstinspires.ftc.teamcode.mk3.camera.CamGirl
import org.firstinspires.ftc.teamcode.mk3.camera.ConePipeline
import org.firstinspires.ftc.teamcode.mk3.tests.CamTest.STREAM
import org.firstinspires.ftc.teamcode.mk3.tests.CamTest.exPerTick
import org.firstinspires.ftc.teamcode.mk3.tests.CamTest.radPerPixel
import java.util.concurrent.TimeUnit
import kotlin.math.abs
import kotlin.math.tan

@Config
object CamTest {
    @JvmField
    var GAIN: Int = 150

    @JvmField
    var EXPOSURE: Int = 10

    @JvmField
    var UPDATE_EXP: Boolean = false

    @JvmField
    var STREAM: Boolean = true

    @JvmField
    var radPerPixel = 0.0009574

    @JvmField
    var exPerTick = 0.115
}

/*
Error: User code threw an uncaught exception: OpenCvCameraException - Camera does not support requested resolution! Supported resolutions are
[640x480], [160x120], [176x144],  [320x176],  [320x240],  [352x288],
[432x240], [544x288], [640x360],  [752x416],  [800x448],  [800x600],
[864x480], [960x544], [960x720], [1024x576], [1184x656], [1280x720],


 */

fun initm(hwm: HardwareMap, s: String?, e: Boolean, r: Boolean): DcMotorEx? { /// Init a motor
    val m = hwm.get(DcMotorEx::class.java, s)
    val mconf = m.motorType.clone()
    mconf.achieveableMaxRPMFraction = 1.0
    m.motorType = mconf
    if (e) {
        m.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        m.mode = DcMotor.RunMode.RUN_USING_ENCODER
    } else {
        m.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }
    m.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    m.direction = if (r) DcMotorSimple.Direction.REVERSE else DcMotorSimple.Direction.FORWARD
    return m
}

fun conversiePerverssa(sextA: Servo, sextB: Servo, p: Double) { /// Handle moving both grabber arm servos
    sextA.position = p
    sextB.position = p + SDIF + (1 - p) * SDIP
}

@TeleOp
class CameraTest : LinearOpMode() {
    override fun runOpMode() {
        /*
        val conePipeline = ConePipeline(ConeHeight, ConeWidth)
        val coneGirl = CamGirl(this, "coneGirl", ConeRotation, ConeWidth, ConeHeight, conePipeline, streaming = STREAM, waitForOpen = true)*/

        val lamprey1 = hardwareMap.get(AnalogInput::class.java, "lamprey");
        val lamprey2 = hardwareMap.get(AnalogInput::class.java, "lamprey2");

        var LG = CamTest.GAIN
        var LE = CamTest.EXPOSURE

        val sHeading = hardwareMap.get(Servo::class.java, "sHeading")
        val sBalans = hardwareMap.get(Servo::class.java, "sBalans")
        val sextA = hardwareMap.get(Servo::class.java, "sextA")
        val sextB = hardwareMap.get(Servo::class.java, "sextB")


        waitForStart()

        //val exPerTick = 0.17
        //val angPerPixel = 0.086
        val camDist = 40
        val distFromWall = 150
        val ext = initm(hardwareMap, "extA", e = true, r = false)

        while (opModeIsActive()) {
            /*
            sHeading.position = SHG
            sBalans.position = SBAG
            conversiePerverssa(sextA, sextB, SAG)

            val monkey = TelemetryPacket()
            val cdist = distFromWall - exPerTick * ext!!.currentPosition - camDist
            val cang = radPerPixel * conePipeline.xoff
            monkey.put("Current camera distance", cdist)
            monkey.put("Current angle", cang)
            val yoff = if (cang >= 0) {
                tan(cang) * cdist
            } else {
                -tan(abs(cang)) * cdist
            }
            monkey.put("Yoff", yoff)
            monkey.put("CurExt", ext.currentPosition)
            FtcDashboard.getInstance().sendTelemetryPacket(monkey)*/

            val ttp = TelemetryPacket()
            ttp.put("L1_MAX", lamprey1.maxVoltage)
            ttp.put("L1_CUR", lamprey1.voltage)
            ttp.put("L2_MAX", lamprey2.maxVoltage)
            ttp.put("L2_CUR", lamprey2.voltage)
            //ttp.put("CONE_PIPELINE_CAM_FPS", coneGirl.camera.fps)
            FtcDashboard.getInstance().sendTelemetryPacket(ttp)
            /*
            if (LG != CamTest.GAIN) {
                coneGirl.camera.gainControl.gain = CamTest.GAIN
                LG = CamTest.GAIN
            }

            if (LE != CamTest.EXPOSURE) {
                coneGirl.camera.exposureControl.setExposure(CamTest.EXPOSURE.toLong(), TimeUnit.MILLISECONDS)
                LE = CamTest.EXPOSURE
            }*/

            sleep(2)
        }

        //coneGirl.stop()
    }
}