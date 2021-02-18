package frc.common.control;

import frc.common.math.MathUtils;

public class PidController {
    private PidConstants constants;

    private double setpoint;

    private boolean continuous = false;
    private boolean scaleOutput = false;
    private double inputRange = Double.POSITIVE_INFINITY;
    private double minInput = Double.NEGATIVE_INFINITY;
    private double maxInput = Double.POSITIVE_INFINITY;
    private double minOutput = Double.NEGATIVE_INFINITY;
    private double maxOutput = Double.POSITIVE_INFINITY;

    public double lastError = Double.NaN;
    public double integralAccum = 0.0;
    private double integralRange = Double.POSITIVE_INFINITY;
    private boolean shouldClearIntegralOnErrorSignChange = false;

    public PidController(PidConstants constants) {
        this.constants = constants;
    }

    public double calculate(double current, double dt) {
        double error = setpoint - current;
        if (continuous) {
            error %= inputRange;
            if (Math.abs(error) > inputRange / 2.0) {
                if (error > 0.0) {
                    error -= inputRange;
                } else {
                    error += inputRange;
                }
            }
        }

        if (shouldClearIntegralOnErrorSignChange && !MathUtils.epsilonEquals(error, Math.copySign(error, integralAccum)) && !MathUtils.epsilonEquals(integralAccum, 0.0)) {
            integralAccum = 0.0;
        }

        double integral = 0.0;
        if (Math.abs(error) < integralRange / 2.0) {
            integral = integralAccum + error * dt;
        }
        integralAccum = integral;

        double derivative = 0.0;
        if (Double.isFinite(lastError)) {
            derivative = (error - lastError) / dt;
        }
        lastError = error;
        if(scaleOutput) {
            return MathUtils.map(constants.p * error + constants.i * integral + constants.d * derivative, minInput, maxInput, minOutput, maxOutput);
        } else {
            return MathUtils.clamp(constants.p * error + constants.i * integral + constants.d * derivative, minOutput, maxOutput);
        }
    }

    public void reset() {
        lastError = Double.NaN;
        integralAccum = 0.0;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public void setContinuous(boolean continuous) {
        this.continuous = continuous;
    }

    public void setInputRange(double minInput, double maxInput) {
        this.inputRange = maxInput - minInput;
        this.minInput = minInput;
        this.maxInput = maxInput;
    }

    public void setIntegralRange(double integralRange) {
        this.integralRange = integralRange;
    }
    //If this is set to true, the output will be scaled, if it's false, it will be clamped.
    public void setScaleOutput(boolean scale) {
        scaleOutput = scale;
    }

    public void setShouldClearIntegralOnErrorSignChange(boolean shouldClearIntegralOnErrorSignChange) {
        this.shouldClearIntegralOnErrorSignChange = shouldClearIntegralOnErrorSignChange;
    }

    /**
     * Sets the output range for the controller. Outputs will be clamped between these two values.
     *
     * @param min the minimum allowable output value
     * @param max the maximum allowable output value
     */
    public void setOutputRange(double min, double max) {
        if (max < min) {
            throw new IllegalArgumentException("Minimum output cannot be greater than maximum output");
        }

        minOutput = min;
        maxOutput = max;
    }
}
