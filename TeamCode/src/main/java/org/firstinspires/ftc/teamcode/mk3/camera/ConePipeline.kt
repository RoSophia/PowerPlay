package org.firstinspires.ftc.teamcode.mk3.camera

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import org.firstinspires.ftc.teamcode.mk3.camera.ConePipeline.Vars.INVERT
import org.firstinspires.ftc.teamcode.mk3.camera.ConePipeline.Vars.TB
import org.firstinspires.ftc.teamcode.mk3.camera.ConePipeline.Vars.TG
import org.firstinspires.ftc.teamcode.mk3.camera.ConePipeline.Vars.TR
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.Point
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline
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

class ConePipeline : OpenCvPipeline() {
    private val frame: Mat = Mat()

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
    }

    private fun boxInFrame(frame: Mat, cb: Box2d) {
        frame.width();
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

    @Config
    object Vars {
        @JvmField
        var WIDTH: Int = 30
        @JvmField
        var UP: Int = 3
        @JvmField
        var LEFT: Int = 6
        @JvmField
        var XOFF: Int = 0
        @JvmField
        var YOFF: Int = 0
        @JvmField
        var INVERT: Boolean = false
        @JvmField
        var ALPHA: Double = 200.0
        @JvmField
        var TB: Double = 140.0
        @JvmField
        var TG: Double = 65.0
        @JvmField
        var TR: Double = 50.0
        @JvmField
        var COL_INDEX: Int = 1
    }

    private fun subm(img: Mat, box: Box2d): Mat {
        return img.submat(box.sy, box.sy + box.height, box.sx, box.sx + box.width)
    }

    private fun check(img: Mat, box: Box2d): DoubleArray {
        val subm = subm(img, box)
        return Core.mean(subm).`val`
    }

    fun is_red(col: DoubleArray): Boolean {
        val b = col[0]
        val g = col[1]
        val r = col[2]
        if (r < g && g < b) {
            val gr = g - r
            val bg = b - g
            if (gr < TG && bg > TB) {
                return true
            }
        }
        return false
    }

    override fun processFrame(input: Mat): Mat {
        input.copyTo(frame)

        if (frame.empty()) {
            return input
        }

        val h = frame.height()
        val w = frame.width()

        val checkLocations = if (INVERT) {
            mklocations(Vars.UP, Vars.LEFT, Vars.WIDTH, Vars.YOFF, Vars.XOFF, h, w)
        } else {
            mklocations(Vars.LEFT, Vars.UP, Vars.WIDTH, Vars.XOFF, Vars.YOFF, w, h)
        }


        var ci = 0
        val ff = Mat()
        frame.copyTo(ff)
        val tp = TelemetryPacket()
        for (element in checkLocations) {
            val vl = check(frame, element)
            if (checkLocations.size <= 6) {
                for (i in vl.indices) {
                    tp.put("val$ci _$i", vl[i])
                }
            }
            if (is_red(vl)) {
                draw(ff, element, 255.0, vl[Vars.COL_INDEX])
            } else {
                draw(ff, element, 0.0, vl[Vars.COL_INDEX])
            }
            ++ci
        }
        FtcDashboard.getInstance().sendTelemetryPacket(tp)

        // results = arrayOf(Box2d())
        ++framec
        return ff
    }
}