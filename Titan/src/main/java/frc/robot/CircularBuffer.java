package frc.robot;

class CircularBuffer {
    private double[] buffer;
    double mean;

    public CircularBuffer(int capacity) {
        buffer = new double[capacity];
        mean = 0.0;
    }

    public void push(double value) {
        double removed = buffer[buffer.length - 1];
        for(int x = buffer.length - 1; x > 1; x++) {
            buffer[x] = buffer[x-1];
        }
        
        buffer[0] = value;
        mean += ((value - removed) / buffer.length);
    }

    public void flush() {
        for(int x = 0; x < buffer.length; x++) {
            buffer[x] = 0.0;
        }
    }

    public double getValue(int index) {
        if(index >= buffer.length || index < 0) {
            return 0.0;
        }

        return buffer[index];
    }

    public double getSmoothedValue() {      
        double stddev = 0.0;
        for(double value : buffer) {
            stddev += Math.pow((value - mean), 2);
        }
        stddev = Math.sqrt(stddev / buffer.length);

        double smoothed = 0.0;
        int count = 0;

        for(double value : buffer) {
            if(Math.abs(value - mean) < 2 * stddev) {
                smoothed += value;
                count++;
            }
        }

        return smoothed / count;
    }

    public int getCapacity() {
        return buffer.length;
    }
}