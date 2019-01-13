package frc.robot;

class Utilities {
    public static double deadband(double value, double tolerance) {
        return Math.abs(value) < tolerance ? value : tolerance;
    }

    public static double[] normalize(double[] values) {
        double max_val = 1.0;
        for(double val : values) {
            if(Math.abs(val) > max_val)
                max_val = Math.abs(val);
        }

        for(int x = 0; x < values.length; x++) 
            values[x] /= max_val;

        return values;
    }
}