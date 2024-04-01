// Generated by the protocol buffer compiler. DO NOT EDIT!
// source: modules/common/proto/header.proto

// Generated files should ignore deprecation warnings
@file:Suppress("DEPRECATION")
package apollo.common;

@kotlin.jvm.JvmName("-initializeheader")
public inline fun header(block: apollo.common.HeaderKt.Dsl.() -> kotlin.Unit): apollo.common.HeaderOuterClass.Header =
  apollo.common.HeaderKt.Dsl._create(apollo.common.HeaderOuterClass.Header.newBuilder()).apply { block() }._build()
/**
 * Protobuf type `apollo.common.Header`
 */
public object HeaderKt {
  @kotlin.OptIn(com.google.protobuf.kotlin.OnlyForUseByGeneratedProtoCode::class)
  @com.google.protobuf.kotlin.ProtoDslMarker
  public class Dsl private constructor(
    private val _builder: apollo.common.HeaderOuterClass.Header.Builder
  ) {
    public companion object {
      @kotlin.jvm.JvmSynthetic
      @kotlin.PublishedApi
      internal fun _create(builder: apollo.common.HeaderOuterClass.Header.Builder): Dsl = Dsl(builder)
    }

    @kotlin.jvm.JvmSynthetic
    @kotlin.PublishedApi
    internal fun _build(): apollo.common.HeaderOuterClass.Header = _builder.build()

    /**
     * ```
     * Message publishing time in seconds.
     * ```
     *
     * `optional double timestamp_sec = 1;`
     */
    public var timestampSec: kotlin.Double
      @JvmName("getTimestampSec")
      get() = _builder.getTimestampSec()
      @JvmName("setTimestampSec")
      set(value) {
        _builder.setTimestampSec(value)
      }
    /**
     * ```
     * Message publishing time in seconds.
     * ```
     *
     * `optional double timestamp_sec = 1;`
     */
    public fun clearTimestampSec() {
      _builder.clearTimestampSec()
    }
    /**
     * ```
     * Message publishing time in seconds.
     * ```
     *
     * `optional double timestamp_sec = 1;`
     * @return Whether the timestampSec field is set.
     */
    public fun hasTimestampSec(): kotlin.Boolean {
      return _builder.hasTimestampSec()
    }

    /**
     * ```
     * Module name.
     * ```
     *
     * `optional string module_name = 2;`
     */
    public var moduleName: kotlin.String
      @JvmName("getModuleName")
      get() = _builder.getModuleName()
      @JvmName("setModuleName")
      set(value) {
        _builder.setModuleName(value)
      }
    /**
     * ```
     * Module name.
     * ```
     *
     * `optional string module_name = 2;`
     */
    public fun clearModuleName() {
      _builder.clearModuleName()
    }
    /**
     * ```
     * Module name.
     * ```
     *
     * `optional string module_name = 2;`
     * @return Whether the moduleName field is set.
     */
    public fun hasModuleName(): kotlin.Boolean {
      return _builder.hasModuleName()
    }

    /**
     * ```
     * Sequence number for each message. Each module maintains its own counter for
     * sequence_num, always starting from 1 on boot.
     * ```
     *
     * `optional uint32 sequence_num = 3;`
     */
    public var sequenceNum: kotlin.Int
      @JvmName("getSequenceNum")
      get() = _builder.getSequenceNum()
      @JvmName("setSequenceNum")
      set(value) {
        _builder.setSequenceNum(value)
      }
    /**
     * ```
     * Sequence number for each message. Each module maintains its own counter for
     * sequence_num, always starting from 1 on boot.
     * ```
     *
     * `optional uint32 sequence_num = 3;`
     */
    public fun clearSequenceNum() {
      _builder.clearSequenceNum()
    }
    /**
     * ```
     * Sequence number for each message. Each module maintains its own counter for
     * sequence_num, always starting from 1 on boot.
     * ```
     *
     * `optional uint32 sequence_num = 3;`
     * @return Whether the sequenceNum field is set.
     */
    public fun hasSequenceNum(): kotlin.Boolean {
      return _builder.hasSequenceNum()
    }

    /**
     * ```
     * Lidar Sensor timestamp for nano-second.
     * ```
     *
     * `optional uint64 lidar_timestamp = 4;`
     */
    public var lidarTimestamp: kotlin.Long
      @JvmName("getLidarTimestamp")
      get() = _builder.getLidarTimestamp()
      @JvmName("setLidarTimestamp")
      set(value) {
        _builder.setLidarTimestamp(value)
      }
    /**
     * ```
     * Lidar Sensor timestamp for nano-second.
     * ```
     *
     * `optional uint64 lidar_timestamp = 4;`
     */
    public fun clearLidarTimestamp() {
      _builder.clearLidarTimestamp()
    }
    /**
     * ```
     * Lidar Sensor timestamp for nano-second.
     * ```
     *
     * `optional uint64 lidar_timestamp = 4;`
     * @return Whether the lidarTimestamp field is set.
     */
    public fun hasLidarTimestamp(): kotlin.Boolean {
      return _builder.hasLidarTimestamp()
    }

    /**
     * ```
     * Camera Sensor timestamp for nano-second.
     * ```
     *
     * `optional uint64 camera_timestamp = 5;`
     */
    public var cameraTimestamp: kotlin.Long
      @JvmName("getCameraTimestamp")
      get() = _builder.getCameraTimestamp()
      @JvmName("setCameraTimestamp")
      set(value) {
        _builder.setCameraTimestamp(value)
      }
    /**
     * ```
     * Camera Sensor timestamp for nano-second.
     * ```
     *
     * `optional uint64 camera_timestamp = 5;`
     */
    public fun clearCameraTimestamp() {
      _builder.clearCameraTimestamp()
    }
    /**
     * ```
     * Camera Sensor timestamp for nano-second.
     * ```
     *
     * `optional uint64 camera_timestamp = 5;`
     * @return Whether the cameraTimestamp field is set.
     */
    public fun hasCameraTimestamp(): kotlin.Boolean {
      return _builder.hasCameraTimestamp()
    }

    /**
     * ```
     * Radar Sensor timestamp for nano-second.
     * ```
     *
     * `optional uint64 radar_timestamp = 6;`
     */
    public var radarTimestamp: kotlin.Long
      @JvmName("getRadarTimestamp")
      get() = _builder.getRadarTimestamp()
      @JvmName("setRadarTimestamp")
      set(value) {
        _builder.setRadarTimestamp(value)
      }
    /**
     * ```
     * Radar Sensor timestamp for nano-second.
     * ```
     *
     * `optional uint64 radar_timestamp = 6;`
     */
    public fun clearRadarTimestamp() {
      _builder.clearRadarTimestamp()
    }
    /**
     * ```
     * Radar Sensor timestamp for nano-second.
     * ```
     *
     * `optional uint64 radar_timestamp = 6;`
     * @return Whether the radarTimestamp field is set.
     */
    public fun hasRadarTimestamp(): kotlin.Boolean {
      return _builder.hasRadarTimestamp()
    }

    /**
     * ```
     * data version
     * ```
     *
     * `optional uint32 version = 7 [default = 1];`
     */
    public var version: kotlin.Int
      @JvmName("getVersion")
      get() = _builder.getVersion()
      @JvmName("setVersion")
      set(value) {
        _builder.setVersion(value)
      }
    /**
     * ```
     * data version
     * ```
     *
     * `optional uint32 version = 7 [default = 1];`
     */
    public fun clearVersion() {
      _builder.clearVersion()
    }
    /**
     * ```
     * data version
     * ```
     *
     * `optional uint32 version = 7 [default = 1];`
     * @return Whether the version field is set.
     */
    public fun hasVersion(): kotlin.Boolean {
      return _builder.hasVersion()
    }

    /**
     * `optional .apollo.common.StatusPb status = 8;`
     */
    public var status: apollo.common.ErrorCodeOuterClass.StatusPb
      @JvmName("getStatus")
      get() = _builder.getStatus()
      @JvmName("setStatus")
      set(value) {
        _builder.setStatus(value)
      }
    /**
     * `optional .apollo.common.StatusPb status = 8;`
     */
    public fun clearStatus() {
      _builder.clearStatus()
    }
    /**
     * `optional .apollo.common.StatusPb status = 8;`
     * @return Whether the status field is set.
     */
    public fun hasStatus(): kotlin.Boolean {
      return _builder.hasStatus()
    }
    public val HeaderKt.Dsl.statusOrNull: apollo.common.ErrorCodeOuterClass.StatusPb?
      get() = _builder.statusOrNull

    /**
     * `optional string frame_id = 9;`
     */
    public var frameId: kotlin.String
      @JvmName("getFrameId")
      get() = _builder.getFrameId()
      @JvmName("setFrameId")
      set(value) {
        _builder.setFrameId(value)
      }
    /**
     * `optional string frame_id = 9;`
     */
    public fun clearFrameId() {
      _builder.clearFrameId()
    }
    /**
     * `optional string frame_id = 9;`
     * @return Whether the frameId field is set.
     */
    public fun hasFrameId(): kotlin.Boolean {
      return _builder.hasFrameId()
    }
  }
}
@kotlin.jvm.JvmSynthetic
public inline fun apollo.common.HeaderOuterClass.Header.copy(block: `apollo.common`.HeaderKt.Dsl.() -> kotlin.Unit): apollo.common.HeaderOuterClass.Header =
  `apollo.common`.HeaderKt.Dsl._create(this.toBuilder()).apply { block() }._build()

public val apollo.common.HeaderOuterClass.HeaderOrBuilder.statusOrNull: apollo.common.ErrorCodeOuterClass.StatusPb?
  get() = if (hasStatus()) getStatus() else null

