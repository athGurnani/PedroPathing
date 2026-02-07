package com.pedropathing;

import com.pedropathing.control.FilteredPIDFController;
import com.pedropathing.control.PIDFController;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;


public class PoseLockController {
    private boolean translationalLock = false, headingLock = false;
    private Pose lockPose = new Pose();
    private PIDFController headingPIDF;
    private FilteredPIDFController translationalPIDF;
    private Vector translationalCorrection = new Vector();
    private double headingCorrection = 0.0;
    private double previousXPower = 0.0, previousYPower = 0.0, previousHeadingPower = 0.0;
    private double currentXPower = 0.0, currentYPower = 0.0, currentHeadingPower = 0.0;

    /**
     * Controls TeleOp heading and translational zero-power lock.
     */
    public PoseLockController(PIDFController headingPIDF, FilteredPIDFController translationalPIDF) {
        this.headingPIDF = headingPIDF;
        this.translationalPIDF = translationalPIDF;
    }

    /**
     * @param currentPose   Current bot pose
     */
    public void update(Pose currentPose, double xPower, double yPower, double headingPower) {
        // Set previous and current powers
        this.previousXPower = this.currentXPower;
        this.currentXPower = xPower;

        this.previousYPower = this.currentYPower;
        this.currentYPower = yPower;

        this.previousHeadingPower = this.currentHeadingPower;
        this.currentHeadingPower = headingPower;


        this.lockPose = this.updateLockPose(currentPose);

        this.updateTranslationalLock(this.lockPose, currentPose);
        headingPIDF.setTargetPosition(lockPose.getHeading());
        headingPIDF.updatePosition(currentPose.getHeading());
        headingCorrection = headingPIDF.run();
    }

    /**
     * @param xPower    X power given to Follower.setTeleopDrive()
     * @return          Translationally locked X power
     */
    public double getPowerX(double xPower) {
        if (xPower == 0) {
            return translationalCorrection.getXComponent();
        }
        else {
            return xPower;
        }
    }

    /**
     * @param yPower    Y power given to Follower.setTeleopDrive()
     * @return          Translationally locked Y power
     */
    public double getPowerY(double yPower) {
        if (yPower == 0) {
            return translationalCorrection.getYComponent();
        }
        else {
            return yPower;
        }
    }

    /**
     *
     * @param headingPower  Heading power given to Follower.setTeleopDrive()
     * @return              Locked heading power
     */
    public double getHeadingPower(double headingPower) {
        if (headingPower == 0) {
            return headingCorrection;
        }
        else {
            return headingPower;
        }
    }

    /**
     * Starts running heading lock
     */
    public void startHeadingLock() {
        headingLock = true;
        headingPIDF.setTargetPosition(lockPose.getHeading());
    }

    /**
     * Ends heading lock
     */
    public void stopHeadingLock() {
        headingLock = false;
        headingPIDF.reset();
    }

    /**
     * Stops translational lock
     */
    public void stopTranslationalLock() {
        translationalLock = false;
        translationalPIDF.reset();
    }

    /**
     * Start translational lock
     */
    //TODO: Allow x/y control individually
    public void startTranslationalLock() {
        translationalLock = true;
        translationalPIDF.setTargetPosition(
                Math.sqrt(
                        Math.pow(lockPose.getX(), 2) + Math.pow(lockPose.getY(), 2)
                )
        );  //Magnitude of polar coordinate
    }
    private void setLockPose(Pose lockPose) {
        this.lockPose = lockPose;
    }

    public Pose getLockPose() {
        return lockPose;
    }

    private Pose updateLockPose(Pose currentPose) {
        Pose newLockPose = this.lockPose;
        if (this.currentXPower == 0 && this.previousXPower != 0) {
            newLockPose = newLockPose.withX(currentPose.getX());
        }

        if (this.currentYPower == 0 && this.previousYPower != 0) {
            newLockPose = newLockPose.withY(currentPose.getY());
        }

        if (this.currentHeadingPower == 0 && this.previousHeadingPower != 0) {
            newLockPose = newLockPose.withHeading(currentPose.getHeading());
        }

        return newLockPose;
    }

    private void updateTranslationalLock(Pose lockPose, Pose currentPose) {
        translationalPIDF.setTargetPosition(
                Math.hypot(
                        lockPose.getX(), lockPose.getY()
                )
        );

        translationalPIDF.updatePosition(
                Math.hypot(
                        currentPose.getX(), currentPose.getY()
                )
        );

        translationalCorrection = new Vector(
                translationalPIDF.run(),
                Math.atan2(
                        currentPose.getY() - lockPose.getY(),   //error between current and target
                        currentPose.getX() - lockPose.getX()
                )
        );
    }
}
