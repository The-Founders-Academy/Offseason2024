package org.firstinspires.ftc.teamcode.subsytems;

public class VisionData {
    private double x;
    private double y;
    private int id;

    public VisionData(double x, double y, int id) {
        this.x = x;
        this.y = y;
        this.id = id;

    }
    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }

    public int getID() {
        return id;
    }

    public void setId(int id) {
        this.id = id;
    }
}
