package frc.robot;

class Utilities {
    public static double deadband(double value, double tolerance) {
        return Math.abs(value) > tolerance ? value : 0.0;
    }

    public static double[] normalize(double[] values) {
        double max_val = values[0];
        boolean normFlag = max_val > 1;
        for(double val : values) {
            if(Math.abs(val) > max_val) {
                max_val = Math.abs(val);
                normFlag = max_val > 1;
            }
        }
        if(normFlag){
        for(int x = 0; x < values.length; x++) 
            values[x] /= max_val;
        }
        return values;
    }

    /**
         * Thanks Dan :) 
		 * Gets rotational error on [-180, 180]
		 * 
		 * @param alpha
		 *            First angle
		 * @param beta
		 *            Second Angle
		 * @return Rotational error
		 */
    public static double rotationalError(double alpha, double beta) {
        double ret = alpha - beta;
        if (ret < -180) {
            ret += 360;
        }
        if (ret > 180) {
            ret -= 360;
        }
        return ret;
    }
}
