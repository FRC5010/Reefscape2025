// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.constants;

/** Used to define persisted constants */
public class GenericPersisted {
  /** The persisted constant type */
  protected String type;
  /** The persisted constant name */
  protected final String name;

  /**
   * Constructor for the GenericPersisted class
   *
   * @param name - The name of the constant
   */
  public GenericPersisted(String name) {
    this.name = name;
  }

  /**
   * Constructor for the GenericPersisted class
   *
   * @param name - the name of the constant
   * @param type - the type of the constant
   */
  public GenericPersisted(String name, String type) {
    this.name = name;
    this.type = type;
  }

  /**
   * Sets the type of the object.
   *
   * @param type the new type of the object
   */
  public void setType(String type) {
    this.type = type;
  }

  /**
   * Returns the type of the object.
   *
   * @return the type of the object as a string
   */
  public String getType() {
    return type;
  }

  /**
   * Returns the name of the object.
   *
   * @return the name of the object as a string
   */
  public String getName() {
    return name;
  }
}
