// package org.firstinspires.ftc.teamcode.Config;
//
// import org.firstinspires.ftc.teamcode.Config.Subsystems.robotHardware;
//
// import com.qualcomm.robotcore.hardware.Gamepad;
// import com.qualcomm.robotcore.hardware.HardwareMap;
// import com.seattlesolvers.solverslib.command.Robot;
// import com.seattlesolvers.solverslib.gamepad.GamepadEx;
//
// public class robot extends Robot {
//  private HardwareMap hardwareMap;
//  private robotHardware hardware;
//  private GamepadEx driver, operator;
//  private boolean isTeleOp = true;
//
//  // Triggers and buttons
//
//  // the constructor with a specified opmode type
//  /**
//   * @param hardwareMap the hardware map of the robot inits as auto
//   */
//  public robot(HardwareMap hardwareMap) {
//    this.hardwareMap = hardwareMap;
//    initAuto();
//  }
//
//  public robot(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
//    this.hardwareMap = hardwareMap;
//    initTele(gamepad1, gamepad2);
//  }
//
//  /*
//   * Initialize teleop or autonomous, depending on which is used
//   */
//  public void initTele(Gamepad gamepad1, Gamepad gamepad2) {
//    // initialize teleop-specific scheduler
//    hardware = new robotHardware(hardwareMap);
//    this.driver = new GamepadEx(gamepad1);
//    this.operator = new GamepadEx(gamepad2);
//    hardware.getFollower().startTeleopDrive();
//    isTeleOp = true;
//    driverTriggers();
//    driverTriggerCommands();
//  }
//
//  public void initAuto() {
//    // initialize auto-specific scheduler
//    hardware = new robotHardware(hardwareMap);
//    isTeleOp = false;
//  }
//
//  private void driverTriggers() {
//    driver.gamepad.setLedColor(0, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
//  }
//
//  private void driverTriggerCommands() {}
//
//  public void read() {
//    if (isTeleOp) {
//      driver.readButtons();
//    }
//    hardware.read();
//  }
//
//  public void loop() {
//    hardware.loop();
//  }
//
//  public void write() {
//    hardware.write();
//  }
//
//  /**
//   * Get the robot hardware subsystem
//   *
//   * @return The robotHardware instance
//   */
//  public robotHardware getHardware() {
//    return hardware;
//  }
// }
