package org.firstinspires.ftc.teamcode.util.Math;

import java.util.Vector;

/**
 * Created by vzyrianov on 5/19/2018
 */

//2D Vector. Components are doubles.
public class Vector2 {
    public double x,
                  y;

    public Vector2() {
        x = 0.0;
        y = 0.0;
    }

    public Vector2(double x, double y){
        this.x = x;
        this.y = y;
    }

    //Sets the vector to equal the cartesian equivalent of the polar vector composed of a length and an angle
    public void setFromPolar(double r, double theta) {
        this.x = Math.cos(theta) * r;
        this.y = Math.sin(theta) * r;
    }
    //Adds the components of a given Vector2 to this Vector2
    public void add(Vector2 vector) {
        x += vector.x;
        y += vector.y;
    }
    //subtracts the components of a given vector from this vector
    public void subtract(Vector2 vector) {
        vector.scalarMultiply(-1);
        add(vector);
    }
    //multiplies x and y by the given value
    public Vector2 scalarMultiply(double a) {
        x *= a;
        y *= a;
        return this;
    }

    //Length of vector
    public double magnitude() { return Math.hypot(x, y); }

    //Dot Product
    public double dot(Vector2 vector) {
        return vector.x * this.x + vector.y * this.y;
    }

    //Cross Product with z parameter equal to zero
    public Vector3 cross(Vector2 vector) {
        Vector3 a = new Vector3();
        Vector3 b = new Vector3();

        a.x = this.x;
        a.y = this.y;

        b.x = vector.x;
        b.y = vector.y;

        Vector3 result = a.cross(b);
        return result;
    }

    //Angular component of the vector if it was converted to polar coordinates
    //Returns angle in radians
    public double angle() {
        return Math.atan2(y, x);
    }

    //Returns angle between two vectors in radians.
    public double angleBetween(Vector2 vector) {
        return Math.acos(this.dot(vector) / (this.magnitude() * vector.magnitude()));
    }

    //Sets x and y rotated by the given angle in radians
    public void rotate(double angle){
        double tempX = x, tempY = y;
        x = Math.cos(angle) * tempX - Math.sin(angle) * tempY;
        y = Math.sin(angle) * tempX + Math.cos(angle) * tempY;
    }

    //creates a Vector3 object from the Vector3 object, excluding one axis
    //axis = 0 returns {x,y,0}, 1 returns {x,0,y}, 2 returns {0,x,y}
    public Vector3 toVector3(int axis){
        return new Vector3(x * Math.min(2-axis,1), x*Math.max(axis-1,0) + y*Math.max(1-axis,0), y*Math.min(axis,1));
    }
    //flips the x and y values of the vector
    public void flipAxes(){
        double temp = x;
        x=y;
        y=temp;
    }

    public String toString(){
        return "(" + x + ", " + y + ")";
    }
}
