package org.frc2910.robot.config;

/**
 *
 * PID default values are all 0.0.
 */
public final class FollowPathConfiguration {
    private double translationKp;
    private double translationKi;
    private double translationKd;
    private double rotationKp;
    private double rotationKi;
    private double rotationKd;

    /**
     * Create a new configuration.
     */
    public FollowPathConfiguration(
            double translationKp,
            double translationKi,
            double translationKd,
            double rotationKp,
            double rotationKi,
            double rotationKd) {
        this.translationKp = translationKp;
        this.translationKi = translationKi;
        this.translationKd = translationKd;
        this.rotationKp = rotationKp;
        this.rotationKi = rotationKi;
        this.rotationKd = rotationKd;
    }

    /**
     * Create a new configuration.
     */
    public FollowPathConfiguration() {
        this(0, 0, 0, 0, 0, 0);
    }

    /**
     * Get the translation proportional gain.
     */
    public double getTranslationKp() {
        return translationKp;
    }

    /**
     * Set the translation proportional gain.
     *
     * @return This <code>FollowPathConfiguration</code>.
     */
    public FollowPathConfiguration withTranslationKp(final double translationKp) {
        this.translationKp = translationKp;

        return this;
    }

    /**
     * Get the translation integral gain.
     */
    public double getTranslationKi() {
        return translationKi;
    }

    /**
     * Set the translation integral gain.
     *
     * @return This <code>FollowPathConfiguration</code>.
     */
    public FollowPathConfiguration withTranslationKi(final double translationKi) {
        this.translationKi = translationKi;

        return this;
    }

    /**
     * Get the translation differential gain.
     */
    public double getTranslationKd() {
        return translationKd;
    }

    /**
     * Set the translation differential gain.
     *
     * @return This <code>FollowPathConfiguration</code>.
     */
    public FollowPathConfiguration withTranslationKd(final double translationKd) {
        this.translationKd = translationKd;

        return this;
    }

    /**
     * Get the rotation proportional gain.
     */
    public double getRotationKp() {
        return rotationKp;
    }

    /**
     * Set the rotation proportional gain.
     *
     * @return This <code>FollowPathConfiguration</code>.
     */
    public FollowPathConfiguration withRotationKp(final double rotationKp) {
        this.rotationKp = rotationKp;

        return this;
    }

    /**
     * Get the rotation integral gain.
     */
    public double getRotationKi() {
        return rotationKi;
    }

    /**
     * Set the rotation integral gain.
     *
     * @return This <code>FollowPathConfiguration</code>.
     */
    public FollowPathConfiguration withRotationKi(final double rotationKi) {
        this.rotationKi = rotationKi;

        return this;
    }

    /**
     * Get the rotation differential gain.
     */
    public double getRotationKd() {
        return rotationKd;
    }

    /**
     * Set the rotation differential gain.
     *
     * @return This <code>FollowPathConfiguration</code>.
     */
    public FollowPathConfiguration withRotationKd(final double rotationKd) {
        this.rotationKd = rotationKd;

        return this;
    }
}
