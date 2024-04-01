// Generated by the protocol buffer compiler. DO NOT EDIT!
// source: modules/perception/proto/perception_obstacle.proto

// Generated files should ignore deprecation warnings
@file:Suppress("DEPRECATION")
package apollo.perception;

@kotlin.jvm.JvmName("-initializelaneMarker")
public inline fun laneMarker(block: apollo.perception.LaneMarkerKt.Dsl.() -> kotlin.Unit): apollo.perception.PerceptionObstacleOuterClass.LaneMarker =
  apollo.perception.LaneMarkerKt.Dsl._create(apollo.perception.PerceptionObstacleOuterClass.LaneMarker.newBuilder()).apply { block() }._build()
/**
 * Protobuf type `apollo.perception.LaneMarker`
 */
public object LaneMarkerKt {
  @kotlin.OptIn(com.google.protobuf.kotlin.OnlyForUseByGeneratedProtoCode::class)
  @com.google.protobuf.kotlin.ProtoDslMarker
  public class Dsl private constructor(
    private val _builder: apollo.perception.PerceptionObstacleOuterClass.LaneMarker.Builder
  ) {
    public companion object {
      @kotlin.jvm.JvmSynthetic
      @kotlin.PublishedApi
      internal fun _create(builder: apollo.perception.PerceptionObstacleOuterClass.LaneMarker.Builder): Dsl = Dsl(builder)
    }

    @kotlin.jvm.JvmSynthetic
    @kotlin.PublishedApi
    internal fun _build(): apollo.perception.PerceptionObstacleOuterClass.LaneMarker = _builder.build()

    /**
     * `optional .apollo.hdmap.LaneBoundaryType.Type lane_type = 1;`
     */
    public var laneType: apollo.hdmap.MapLane.LaneBoundaryType.Type
      @JvmName("getLaneType")
      get() = _builder.getLaneType()
      @JvmName("setLaneType")
      set(value) {
        _builder.setLaneType(value)
      }
    /**
     * `optional .apollo.hdmap.LaneBoundaryType.Type lane_type = 1;`
     */
    public fun clearLaneType() {
      _builder.clearLaneType()
    }
    /**
     * `optional .apollo.hdmap.LaneBoundaryType.Type lane_type = 1;`
     * @return Whether the laneType field is set.
     */
    public fun hasLaneType(): kotlin.Boolean {
      return _builder.hasLaneType()
    }

    /**
     * ```
     * range = [0,1]; 1 = the best quality
     * ```
     *
     * `optional double quality = 2;`
     */
    public var quality: kotlin.Double
      @JvmName("getQuality")
      get() = _builder.getQuality()
      @JvmName("setQuality")
      set(value) {
        _builder.setQuality(value)
      }
    /**
     * ```
     * range = [0,1]; 1 = the best quality
     * ```
     *
     * `optional double quality = 2;`
     */
    public fun clearQuality() {
      _builder.clearQuality()
    }
    /**
     * ```
     * range = [0,1]; 1 = the best quality
     * ```
     *
     * `optional double quality = 2;`
     * @return Whether the quality field is set.
     */
    public fun hasQuality(): kotlin.Boolean {
      return _builder.hasQuality()
    }

    /**
     * `optional int32 model_degree = 3;`
     */
    public var modelDegree: kotlin.Int
      @JvmName("getModelDegree")
      get() = _builder.getModelDegree()
      @JvmName("setModelDegree")
      set(value) {
        _builder.setModelDegree(value)
      }
    /**
     * `optional int32 model_degree = 3;`
     */
    public fun clearModelDegree() {
      _builder.clearModelDegree()
    }
    /**
     * `optional int32 model_degree = 3;`
     * @return Whether the modelDegree field is set.
     */
    public fun hasModelDegree(): kotlin.Boolean {
      return _builder.hasModelDegree()
    }

    /**
     * ```
     * equation X = c3 * Z^3 + c2 * Z^2 + c1 * Z + c0
     * ```
     *
     * `optional double c0_position = 4;`
     */
    public var c0Position: kotlin.Double
      @JvmName("getC0Position")
      get() = _builder.getC0Position()
      @JvmName("setC0Position")
      set(value) {
        _builder.setC0Position(value)
      }
    /**
     * ```
     * equation X = c3 * Z^3 + c2 * Z^2 + c1 * Z + c0
     * ```
     *
     * `optional double c0_position = 4;`
     */
    public fun clearC0Position() {
      _builder.clearC0Position()
    }
    /**
     * ```
     * equation X = c3 * Z^3 + c2 * Z^2 + c1 * Z + c0
     * ```
     *
     * `optional double c0_position = 4;`
     * @return Whether the c0Position field is set.
     */
    public fun hasC0Position(): kotlin.Boolean {
      return _builder.hasC0Position()
    }

    /**
     * `optional double c1_heading_angle = 5;`
     */
    public var c1HeadingAngle: kotlin.Double
      @JvmName("getC1HeadingAngle")
      get() = _builder.getC1HeadingAngle()
      @JvmName("setC1HeadingAngle")
      set(value) {
        _builder.setC1HeadingAngle(value)
      }
    /**
     * `optional double c1_heading_angle = 5;`
     */
    public fun clearC1HeadingAngle() {
      _builder.clearC1HeadingAngle()
    }
    /**
     * `optional double c1_heading_angle = 5;`
     * @return Whether the c1HeadingAngle field is set.
     */
    public fun hasC1HeadingAngle(): kotlin.Boolean {
      return _builder.hasC1HeadingAngle()
    }

    /**
     * `optional double c2_curvature = 6;`
     */
    public var c2Curvature: kotlin.Double
      @JvmName("getC2Curvature")
      get() = _builder.getC2Curvature()
      @JvmName("setC2Curvature")
      set(value) {
        _builder.setC2Curvature(value)
      }
    /**
     * `optional double c2_curvature = 6;`
     */
    public fun clearC2Curvature() {
      _builder.clearC2Curvature()
    }
    /**
     * `optional double c2_curvature = 6;`
     * @return Whether the c2Curvature field is set.
     */
    public fun hasC2Curvature(): kotlin.Boolean {
      return _builder.hasC2Curvature()
    }

    /**
     * `optional double c3_curvature_derivative = 7;`
     */
    public var c3CurvatureDerivative: kotlin.Double
      @JvmName("getC3CurvatureDerivative")
      get() = _builder.getC3CurvatureDerivative()
      @JvmName("setC3CurvatureDerivative")
      set(value) {
        _builder.setC3CurvatureDerivative(value)
      }
    /**
     * `optional double c3_curvature_derivative = 7;`
     */
    public fun clearC3CurvatureDerivative() {
      _builder.clearC3CurvatureDerivative()
    }
    /**
     * `optional double c3_curvature_derivative = 7;`
     * @return Whether the c3CurvatureDerivative field is set.
     */
    public fun hasC3CurvatureDerivative(): kotlin.Boolean {
      return _builder.hasC3CurvatureDerivative()
    }

    /**
     * `optional double view_range = 8;`
     */
    public var viewRange: kotlin.Double
      @JvmName("getViewRange")
      get() = _builder.getViewRange()
      @JvmName("setViewRange")
      set(value) {
        _builder.setViewRange(value)
      }
    /**
     * `optional double view_range = 8;`
     */
    public fun clearViewRange() {
      _builder.clearViewRange()
    }
    /**
     * `optional double view_range = 8;`
     * @return Whether the viewRange field is set.
     */
    public fun hasViewRange(): kotlin.Boolean {
      return _builder.hasViewRange()
    }

    /**
     * `optional double longitude_start = 9;`
     */
    public var longitudeStart: kotlin.Double
      @JvmName("getLongitudeStart")
      get() = _builder.getLongitudeStart()
      @JvmName("setLongitudeStart")
      set(value) {
        _builder.setLongitudeStart(value)
      }
    /**
     * `optional double longitude_start = 9;`
     */
    public fun clearLongitudeStart() {
      _builder.clearLongitudeStart()
    }
    /**
     * `optional double longitude_start = 9;`
     * @return Whether the longitudeStart field is set.
     */
    public fun hasLongitudeStart(): kotlin.Boolean {
      return _builder.hasLongitudeStart()
    }

    /**
     * `optional double longitude_end = 10;`
     */
    public var longitudeEnd: kotlin.Double
      @JvmName("getLongitudeEnd")
      get() = _builder.getLongitudeEnd()
      @JvmName("setLongitudeEnd")
      set(value) {
        _builder.setLongitudeEnd(value)
      }
    /**
     * `optional double longitude_end = 10;`
     */
    public fun clearLongitudeEnd() {
      _builder.clearLongitudeEnd()
    }
    /**
     * `optional double longitude_end = 10;`
     * @return Whether the longitudeEnd field is set.
     */
    public fun hasLongitudeEnd(): kotlin.Boolean {
      return _builder.hasLongitudeEnd()
    }
  }
}
@kotlin.jvm.JvmSynthetic
public inline fun apollo.perception.PerceptionObstacleOuterClass.LaneMarker.copy(block: `apollo.perception`.LaneMarkerKt.Dsl.() -> kotlin.Unit): apollo.perception.PerceptionObstacleOuterClass.LaneMarker =
  `apollo.perception`.LaneMarkerKt.Dsl._create(this.toBuilder()).apply { block() }._build()

