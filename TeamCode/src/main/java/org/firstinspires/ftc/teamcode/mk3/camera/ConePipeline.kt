package org.firstinspires.ftc.teamcode.mk3.camera

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import org.firstinspires.ftc.teamcode.mk3.camera.ConePipeline.Vars.BC
import org.firstinspires.ftc.teamcode.mk3.camera.ConePipeline.Vars.DRAW_BOXES
import org.firstinspires.ftc.teamcode.mk3.camera.ConePipeline.Vars.INVERT
import org.firstinspires.ftc.teamcode.mk3.camera.ConePipeline.Vars.RC
import org.firstinspires.ftc.teamcode.mk3.camera.ConePipeline.Vars.TB
import org.firstinspires.ftc.teamcode.mk3.camera.ConePipeline.Vars.TR
import org.firstinspires.ftc.teamcode.mk3.camera.ConePipeline.Vars.USE_TELE
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.Point
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline
import kotlin.math.abs
import kotlin.math.sqrt

class Box2d(x: Int, y: Int, w: Int, h: Int) : Comparable<Box2d> {
    var sx: Int = 0
    var sy: Int = 0
    var width: Int = 0
    var height: Int = 0

    constructor() : this(0, 0, 0, 0)

    init {
        sx = x
        sy = y
        width = w
        height = h
    }

    fun gdist(): Double {
        val xd = sx.toDouble() + width.toDouble() / 2
        val yd = sy.toDouble() + height.toDouble() / 2
        return sqrt(xd * xd + yd * yd)
    }

    override fun compareTo(other: Box2d): Int {
        return if (gdist() < other.gdist()) {
            1
        } else {
            -1
        }
    }
}

class ConePipeline(height: Int, width: Int) : OpenCvPipeline() {
    private val frame: Mat = Mat()

    var xoff: Double = 0.0

    var results: Array<Box2d> = arrayOf(Box2d())

    private fun mklocations(up: Int, left: Int, width: Int, xoff: Int, yoff: Int, h: Int, w: Int): Array<Box2d> {
        val len: Int = (up * 2 - 1) * (left * 2 - 1)
        val res = Array(len) { Box2d() }
        var ci = 0
        for (i in -(up - 1) until up) {
            for (j in -(left - 1) until left) {
                res[ci] = Box2d(i * width + h / 2 - width / 2 + xoff, j * width + w / 2 - width / 2 + yoff, width, width)
                ++ci
            }
        }

        results.sort()
        return res
        TODO("MAKE THIS RUN ONLY ONCE")
    }

    private fun draw(frame: Mat, cb: Box2d, vl: Double, ll: Double) {
        val p1 = Point(cb.sx.toDouble(),
                cb.sy.toDouble())
        val p3 = Point(cb.sx.toDouble() + cb.width.toDouble(),
                cb.sy.toDouble() + cb.height.toDouble())

        val red = Scalar(ll, ll, ll, Vars.ALPHA)
        val col = Scalar(vl, vl, vl, Vars.ALPHA)

        Imgproc.rectangle(frame, p1, p3, red, 7)
        Imgproc.rectangle(frame, p1, p3, col, -1)
    }

    var framec = 0

    object Vars {
        @JvmField
        var WIDTH: Int = 15

        @JvmField
        var LUP: Int = 9

        @JvmField
        var LUT: Int = 18

        @JvmField
        var XOFF: Int = 0

        @JvmField
        var YOFF: Int = 0

        @JvmField
        var INVERT: Boolean = false

        @JvmField
        var ALPHA: Double = 200.0

        @JvmField
        var TB: Double = 75.0

        @JvmField
        var TR: Double = 25.0

        @JvmField
        var COL_INDEX: Int = 1

        @JvmField
        var TARGET_BRIGHTNESS: Double = 1.0

        var RC: Double = 0.5
        var BC: Double = 1.0

        @JvmField
        var TP: Double = 1.0

        @JvmField
        var DRAW_BOXES: Boolean = true

        @JvmField
        var USE_TELE: Boolean = false
    }

    private fun subm(img: Mat, box: Box2d): Mat {
        return img.submat(box.sy, box.sy + box.height, box.sx, box.sx + box.width)
    }

    private fun check(img: Mat, box: Box2d): DoubleArray {
        val subm = subm(img, box)
        return Core.mean(subm).`val`
    }

    private fun isRed(col: DoubleArray): Boolean {
        val b = col[0]
        val g = col[1]
        val r = col[2]
        val rcoef = (-(r / 255) + 1.5) * RC
        val bg = b - g
        val bcoef = (-(b / 255) + 1.5) * BC

        if (USE_TELE) {
            val tp = TelemetryPacket()
            tp.put("abs(r - g)", abs(r - g))
            tp.put("rcoef", rcoef)
            tp.put("abs(r - g) * rcoef", abs(r - g) * rcoef)

            tp.put("bg", bg)
            tp.put("bcoef", bcoef)
            tp.put("bg * bcoef", bg * bcoef)

            FtcDashboard.getInstance().sendTelemetryPacket(tp)
        }

        if (abs(r - g) * rcoef < TR) {
            if (bg > TB) {
                return true
            }
        }
        return false
    }

    fun getRecommendedExposureDifference(): Int {
        val mc = Core.mean(frame)
        val mbrightness = mc.`val`[0] * 0.0722 + mc.`val`[1] * 0.7152 + mc.`val`[2] * 0.2126
        val tp = TelemetryPacket()
        tp.put("MED_BRIGHTNESS", mbrightness)
        tp.put("UpdatedBright", ((Vars.TARGET_BRIGHTNESS - mbrightness) * Vars.TP).toInt())
        FtcDashboard.getInstance().sendTelemetryPacket(tp)
        return ((Vars.TARGET_BRIGHTNESS - mbrightness) * Vars.TP).toInt()
    }

    var checkLocations: Array<Box2d> = if (INVERT) {
        mklocations(Vars.LUP, Vars.LUT, Vars.WIDTH, Vars.YOFF, Vars.XOFF, height, width)
    } else {
        mklocations(Vars.LUT, Vars.LUP, Vars.WIDTH, Vars.XOFF, Vars.YOFF, width, height)
    }

    override fun processFrame(input: Mat): Mat {
        input.copyTo(frame)

        if (frame.empty()) {
            return input
        }

        val ff = Mat()
        frame.copyTo(ff)
        val tp = TelemetryPacket()
        var medXS = 0
        var medYS = 0
        var redc = 0
        for ((ci, element) in checkLocations.withIndex()) {
            val vl = check(frame, element)
            if (checkLocations.size <= 6 && USE_TELE) {
                for (i in vl.indices) {
                    tp.put("val$ci _$i", vl[i])
                }
            }
            if (isRed(vl)) {
                medXS += element.sx + element.width / 2
                medYS += element.sy + element.height / 2
                ++redc
                if (DRAW_BOXES) {
                    draw(ff, element, 255.0, vl[Vars.COL_INDEX])
                }
            } else {
                if (DRAW_BOXES) {
                    draw(ff, element, 0.0, vl[Vars.COL_INDEX])
                }
            }
        }
        if (checkLocations.size <= 6 && USE_TELE) {
            FtcDashboard.getInstance().sendTelemetryPacket(tp)
        }

        val w = frame.width()
        val h = frame.height()
        val medX = if (redc > 0) {
            medXS.toDouble() / redc.toDouble()
        } else {
            w / 2.0
        }
        val medY = if (redc > 0) {
            medYS.toDouble() / redc.toDouble()
        } else {
            h / 2.0
        }
        val p1 = Point(w / 2.0, h / 2.0)
        val p2 = Point(medX, medY)
        Imgproc.line(ff, p1, p2, Scalar(255.0, 0.0, 0.0, 255.0), 6)

        xoff = ((medX - (w / 2.0)) / w) * 2

        ++framec
        return ff
    }
}