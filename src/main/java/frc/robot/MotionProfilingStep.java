package frc.robot;

public class MotionProfilingStep {
    public String name;
    public int stepNumber;
    public double positionTarget;
    public double velocityMax;
    public double accelMax;
    public double marginOfError;
    
    
    public MotionProfilingStep(String name, int stepNumber, double positionTarget, double velocityMax, double accelMax,
            double marginOfError) {
        this.name = name;
        this.stepNumber = stepNumber;
        this.positionTarget = positionTarget;
        this.velocityMax = velocityMax;
        this.accelMax = accelMax;
        this.marginOfError = marginOfError;
    }

}
