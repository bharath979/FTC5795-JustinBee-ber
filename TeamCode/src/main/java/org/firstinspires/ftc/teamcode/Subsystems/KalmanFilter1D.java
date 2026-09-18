package org.firstinspires.ftc.teamcode.Subsystems;

/**
 * Minimal scalar Kalman filter (constant-value model) used to smooth a single noisy
 * measurement stream, e.g. one field of an AprilTag pose.
 */
public class KalmanFilter1D {
    private final double processNoise;
    private final double measurementNoise;
    private double estimate;
    private double errorCovariance;
    private boolean initialized = false;

    public KalmanFilter1D(double processNoise, double measurementNoise) {
        this.processNoise = processNoise;
        this.measurementNoise = measurementNoise;
        this.errorCovariance = 1.0;
    }

    public double update(double measurement) {
        if (!initialized) {
            estimate = measurement;
            initialized = true;
            return estimate;
        }

        // Predict
        errorCovariance += processNoise;

        // Update
        double kalmanGain = errorCovariance / (errorCovariance + measurementNoise);
        estimate += kalmanGain * (measurement - estimate);
        errorCovariance *= (1 - kalmanGain);

        return estimate;
    }

    public double getEstimate() {
        return estimate;
    }

    public void reset() {
        initialized = false;
        errorCovariance = 1.0;
    }
}
