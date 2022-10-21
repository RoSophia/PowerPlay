package org.firstinspires.ftc.teamcode.husk;

public class EElement {
    public int x0;
    public int y0;
    public int x1;
    public int y1;
    public int ID;
    public int learned;

    enum EType {ARROW, BLOCK};
    public EType type;

    public EElement(int xt, int yt, int xh, int yh, int id, boolean ct) {
        x0 = xt;
        y0 = yt;
        x1 = xh;
        y1 = yh;
        ID = id;
        learned = ID > 0 ? 1 : 0;
        type = ct ? EType.BLOCK : EType.ARROW;
    }
}