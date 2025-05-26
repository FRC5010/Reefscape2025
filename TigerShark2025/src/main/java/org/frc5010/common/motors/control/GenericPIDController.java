package org.frc5010.common.motors.control;

import edu.wpi.first.math.controller.PIDController;
import org.frc5010.common.motors.PIDController5010;

public abstract class GenericPIDController implements PIDController5010 {
  protected PIDController pidController = new PIDController(0, 0, 0);

  @Override
  public void resetController(double position, double velocity) {}

  @Override
  public double getReferenceVelocity() {
    return 0;
  }

  public double calculateControlEffort(double current) {
    pidController.setPID(getP(), getI(), getD());
    pidController.setIZone(getIZone());
    pidController.setSetpoint(getReference());
    pidController.setTolerance(getTolerance());
    double control = pidController.calculate(current);
    return control + getF();
  }
}
