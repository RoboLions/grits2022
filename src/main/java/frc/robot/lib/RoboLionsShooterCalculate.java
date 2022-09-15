package frc.robot.lib;

public class RoboLionsShooterCalculate {

    public static double y = 0;
    public static double heightOfGoalMeters = 2.44;
    private static final double A = 0;
    // x will be horizontal distance
    // A will be the vertical distance of the goal
    public static double calculate(double x) {
    heightOfGoalMeters = A;
    y = (-1*(16*A*(Math.pow(x, 2)) + 16*(Math.pow(A,3))) + 
    Math.sqrt(Math.pow((16*A*(Math.pow(x, 2)) + 16*(Math.pow(A, 3))), 2) - 4*((-16*Math.pow(A, 2))-16*Math.pow(x, 2)))) / 
    (2*((-16*Math.pow(A, 2)) - 16*Math.pow(x, 2)));
    //change the minus to plus if a negative value is returned
    if (y <= 2.9) { //y Min in meters
        y = 2.9;
    }
    return y; 
    }
}
