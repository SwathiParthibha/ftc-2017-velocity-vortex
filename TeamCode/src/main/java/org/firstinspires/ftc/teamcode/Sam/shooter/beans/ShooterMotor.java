package org.firstinspires.ftc.teamcode.Sam.shooter.beans;

public class ShooterMotor {
    private Enum name;
    private double rpm = 0D;

    public Enum getName() {
        return name;
    }

    public void setName(Enum name) {
        this.name = name;
    }

    public double getRpm() {
        return rpm;
    }

    public void setRpm(double rpm) {
        this.rpm = rpm;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        ShooterMotor shooterMotor = (ShooterMotor) o;

        return name != null ? name.equals(shooterMotor.name) : shooterMotor.name == null;

    }

    @Override
    public int hashCode() {
        return name != null ? name.hashCode() : 0;
    }
}
