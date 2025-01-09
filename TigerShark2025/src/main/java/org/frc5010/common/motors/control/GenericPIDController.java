package org.frc5010.common.motors.control;

import edu.wpi.first.math.controller.PIDController;
import org.frc5010.common.motors.PIDController5010;

public abstract class GenericPIDController implements PIDController5010 {
  protected PIDController controller = new PIDController(0, 0, 0);

  public double calculateControlEffort(double current) {
    controller.setPID(getP(), getI(), getD());
    controller.setIZone(getIZone());
    controller.setSetpoint(getReference());
    double control = controller.calculate(current);
    return control + getF();
  }
}
