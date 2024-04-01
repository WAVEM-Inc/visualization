// Generated by the protocol buffer compiler. DO NOT EDIT!
// source: modules/essys/dashboard.proto

// Generated files should ignore deprecation warnings
@file:Suppress("DEPRECATION")
package essys_middle;

@kotlin.jvm.JvmName("-initializevehicleSignal")
public inline fun vehicleSignal(block: essys_middle.VehicleSignalKt.Dsl.() -> kotlin.Unit): essys_middle.Dashboard.VehicleSignal =
  essys_middle.VehicleSignalKt.Dsl._create(essys_middle.Dashboard.VehicleSignal.newBuilder()).apply { block() }._build()
/**
 * Protobuf type `essys_middle.VehicleSignal`
 */
public object VehicleSignalKt {
  @kotlin.OptIn(com.google.protobuf.kotlin.OnlyForUseByGeneratedProtoCode::class)
  @com.google.protobuf.kotlin.ProtoDslMarker
  public class Dsl private constructor(
    private val _builder: essys_middle.Dashboard.VehicleSignal.Builder
  ) {
    public companion object {
      @kotlin.jvm.JvmSynthetic
      @kotlin.PublishedApi
      internal fun _create(builder: essys_middle.Dashboard.VehicleSignal.Builder): Dsl = Dsl(builder)
    }

    @kotlin.jvm.JvmSynthetic
    @kotlin.PublishedApi
    internal fun _build(): essys_middle.Dashboard.VehicleSignal = _builder.build()

    /**
     * `optional .essys_middle.GearPosition gear_position = 1;`
     */
    public var gearPosition: essys_middle.Dashboard.GearPosition
      @JvmName("getGearPosition")
      get() = _builder.getGearPosition()
      @JvmName("setGearPosition")
      set(value) {
        _builder.setGearPosition(value)
      }
    /**
     * `optional .essys_middle.GearPosition gear_position = 1;`
     */
    public fun clearGearPosition() {
      _builder.clearGearPosition()
    }
    /**
     * `optional .essys_middle.GearPosition gear_position = 1;`
     * @return Whether the gearPosition field is set.
     */
    public fun hasGearPosition(): kotlin.Boolean {
      return _builder.hasGearPosition()
    }

    /**
     * `optional double yaw_rate = 2;`
     */
    public var yawRate: kotlin.Double
      @JvmName("getYawRate")
      get() = _builder.getYawRate()
      @JvmName("setYawRate")
      set(value) {
        _builder.setYawRate(value)
      }
    /**
     * `optional double yaw_rate = 2;`
     */
    public fun clearYawRate() {
      _builder.clearYawRate()
    }
    /**
     * `optional double yaw_rate = 2;`
     * @return Whether the yawRate field is set.
     */
    public fun hasYawRate(): kotlin.Boolean {
      return _builder.hasYawRate()
    }

    /**
     * `optional double speed = 3;`
     */
    public var speed: kotlin.Double
      @JvmName("getSpeed")
      get() = _builder.getSpeed()
      @JvmName("setSpeed")
      set(value) {
        _builder.setSpeed(value)
      }
    /**
     * `optional double speed = 3;`
     */
    public fun clearSpeed() {
      _builder.clearSpeed()
    }
    /**
     * `optional double speed = 3;`
     * @return Whether the speed field is set.
     */
    public fun hasSpeed(): kotlin.Boolean {
      return _builder.hasSpeed()
    }

    /**
     * `optional double accelerator = 4;`
     */
    public var accelerator: kotlin.Double
      @JvmName("getAccelerator")
      get() = _builder.getAccelerator()
      @JvmName("setAccelerator")
      set(value) {
        _builder.setAccelerator(value)
      }
    /**
     * `optional double accelerator = 4;`
     */
    public fun clearAccelerator() {
      _builder.clearAccelerator()
    }
    /**
     * `optional double accelerator = 4;`
     * @return Whether the accelerator field is set.
     */
    public fun hasAccelerator(): kotlin.Boolean {
      return _builder.hasAccelerator()
    }

    /**
     * `optional double steering_angle = 5;`
     */
    public var steeringAngle: kotlin.Double
      @JvmName("getSteeringAngle")
      get() = _builder.getSteeringAngle()
      @JvmName("setSteeringAngle")
      set(value) {
        _builder.setSteeringAngle(value)
      }
    /**
     * `optional double steering_angle = 5;`
     */
    public fun clearSteeringAngle() {
      _builder.clearSteeringAngle()
    }
    /**
     * `optional double steering_angle = 5;`
     * @return Whether the steeringAngle field is set.
     */
    public fun hasSteeringAngle(): kotlin.Boolean {
      return _builder.hasSteeringAngle()
    }

    /**
     * `optional double steering_rotation_speed = 6;`
     */
    public var steeringRotationSpeed: kotlin.Double
      @JvmName("getSteeringRotationSpeed")
      get() = _builder.getSteeringRotationSpeed()
      @JvmName("setSteeringRotationSpeed")
      set(value) {
        _builder.setSteeringRotationSpeed(value)
      }
    /**
     * `optional double steering_rotation_speed = 6;`
     */
    public fun clearSteeringRotationSpeed() {
      _builder.clearSteeringRotationSpeed()
    }
    /**
     * `optional double steering_rotation_speed = 6;`
     * @return Whether the steeringRotationSpeed field is set.
     */
    public fun hasSteeringRotationSpeed(): kotlin.Boolean {
      return _builder.hasSteeringRotationSpeed()
    }

    /**
     * `optional double brake_pressure = 7;`
     */
    public var brakePressure: kotlin.Double
      @JvmName("getBrakePressure")
      get() = _builder.getBrakePressure()
      @JvmName("setBrakePressure")
      set(value) {
        _builder.setBrakePressure(value)
      }
    /**
     * `optional double brake_pressure = 7;`
     */
    public fun clearBrakePressure() {
      _builder.clearBrakePressure()
    }
    /**
     * `optional double brake_pressure = 7;`
     * @return Whether the brakePressure field is set.
     */
    public fun hasBrakePressure(): kotlin.Boolean {
      return _builder.hasBrakePressure()
    }

    /**
     * `optional double brake_pressure_feedback = 8;`
     */
    public var brakePressureFeedback: kotlin.Double
      @JvmName("getBrakePressureFeedback")
      get() = _builder.getBrakePressureFeedback()
      @JvmName("setBrakePressureFeedback")
      set(value) {
        _builder.setBrakePressureFeedback(value)
      }
    /**
     * `optional double brake_pressure_feedback = 8;`
     */
    public fun clearBrakePressureFeedback() {
      _builder.clearBrakePressureFeedback()
    }
    /**
     * `optional double brake_pressure_feedback = 8;`
     * @return Whether the brakePressureFeedback field is set.
     */
    public fun hasBrakePressureFeedback(): kotlin.Boolean {
      return _builder.hasBrakePressureFeedback()
    }

    /**
     * `optional .essys_middle.TurnSignal turn_signal = 9;`
     */
    public var turnSignal: essys_middle.Dashboard.TurnSignal
      @JvmName("getTurnSignal")
      get() = _builder.getTurnSignal()
      @JvmName("setTurnSignal")
      set(value) {
        _builder.setTurnSignal(value)
      }
    /**
     * `optional .essys_middle.TurnSignal turn_signal = 9;`
     */
    public fun clearTurnSignal() {
      _builder.clearTurnSignal()
    }
    /**
     * `optional .essys_middle.TurnSignal turn_signal = 9;`
     * @return Whether the turnSignal field is set.
     */
    public fun hasTurnSignal(): kotlin.Boolean {
      return _builder.hasTurnSignal()
    }

    /**
     * `optional .essys_middle.Light head_light = 10;`
     */
    public var headLight: essys_middle.Dashboard.Light
      @JvmName("getHeadLight")
      get() = _builder.getHeadLight()
      @JvmName("setHeadLight")
      set(value) {
        _builder.setHeadLight(value)
      }
    /**
     * `optional .essys_middle.Light head_light = 10;`
     */
    public fun clearHeadLight() {
      _builder.clearHeadLight()
    }
    /**
     * `optional .essys_middle.Light head_light = 10;`
     * @return Whether the headLight field is set.
     */
    public fun hasHeadLight(): kotlin.Boolean {
      return _builder.hasHeadLight()
    }

    /**
     * `optional double wheel_speed = 11;`
     */
    public var wheelSpeed: kotlin.Double
      @JvmName("getWheelSpeed")
      get() = _builder.getWheelSpeed()
      @JvmName("setWheelSpeed")
      set(value) {
        _builder.setWheelSpeed(value)
      }
    /**
     * `optional double wheel_speed = 11;`
     */
    public fun clearWheelSpeed() {
      _builder.clearWheelSpeed()
    }
    /**
     * `optional double wheel_speed = 11;`
     * @return Whether the wheelSpeed field is set.
     */
    public fun hasWheelSpeed(): kotlin.Boolean {
      return _builder.hasWheelSpeed()
    }

    /**
     * `optional .essys_middle.DoorLock door_lock = 12;`
     */
    public var doorLock: essys_middle.Dashboard.DoorLock
      @JvmName("getDoorLock")
      get() = _builder.getDoorLock()
      @JvmName("setDoorLock")
      set(value) {
        _builder.setDoorLock(value)
      }
    /**
     * `optional .essys_middle.DoorLock door_lock = 12;`
     */
    public fun clearDoorLock() {
      _builder.clearDoorLock()
    }
    /**
     * `optional .essys_middle.DoorLock door_lock = 12;`
     * @return Whether the doorLock field is set.
     */
    public fun hasDoorLock(): kotlin.Boolean {
      return _builder.hasDoorLock()
    }

    /**
     * `optional .essys_middle.DoorOpen door_open = 13;`
     */
    public var doorOpen: essys_middle.Dashboard.DoorOpen
      @JvmName("getDoorOpen")
      get() = _builder.getDoorOpen()
      @JvmName("setDoorOpen")
      set(value) {
        _builder.setDoorOpen(value)
      }
    /**
     * `optional .essys_middle.DoorOpen door_open = 13;`
     */
    public fun clearDoorOpen() {
      _builder.clearDoorOpen()
    }
    /**
     * `optional .essys_middle.DoorOpen door_open = 13;`
     * @return Whether the doorOpen field is set.
     */
    public fun hasDoorOpen(): kotlin.Boolean {
      return _builder.hasDoorOpen()
    }

    /**
     * `optional string aeb_trigger = 14;`
     */
    public var aebTrigger: kotlin.String
      @JvmName("getAebTrigger")
      get() = _builder.getAebTrigger()
      @JvmName("setAebTrigger")
      set(value) {
        _builder.setAebTrigger(value)
      }
    /**
     * `optional string aeb_trigger = 14;`
     */
    public fun clearAebTrigger() {
      _builder.clearAebTrigger()
    }
    /**
     * `optional string aeb_trigger = 14;`
     * @return Whether the aebTrigger field is set.
     */
    public fun hasAebTrigger(): kotlin.Boolean {
      return _builder.hasAebTrigger()
    }
  }
}
@kotlin.jvm.JvmSynthetic
public inline fun essys_middle.Dashboard.VehicleSignal.copy(block: `essys_middle`.VehicleSignalKt.Dsl.() -> kotlin.Unit): essys_middle.Dashboard.VehicleSignal =
  `essys_middle`.VehicleSignalKt.Dsl._create(this.toBuilder()).apply { block() }._build()

