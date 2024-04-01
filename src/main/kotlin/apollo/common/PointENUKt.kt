// Generated by the protocol buffer compiler. DO NOT EDIT!
// source: modules/common/proto/geometry.proto

// Generated files should ignore deprecation warnings
@file:Suppress("DEPRECATION")
package apollo.common;

@kotlin.jvm.JvmName("-initializepointENU")
public inline fun pointENU(block: apollo.common.PointENUKt.Dsl.() -> kotlin.Unit): apollo.common.Geometry.PointENU =
  apollo.common.PointENUKt.Dsl._create(apollo.common.Geometry.PointENU.newBuilder()).apply { block() }._build()
/**
 * ```
 * A point in the map reference frame. The map defines an origin, whose
 * coordinate is (0, 0, 0).
 * Most modules, including localization, perception, and prediction, generate
 * results based on the map reference frame.
 * Currently, the map uses Universal Transverse Mercator (UTM) projection. See
 * the link below for the definition of map origin.
 * https://en.wikipedia.org/wiki/Universal_Transverse_Mercator_coordinate_system
 * The z field of PointENU can be omitted. If so, it is a 2D location and we do
 * not care its height.
 * ```
 *
 * Protobuf type `apollo.common.PointENU`
 */
public object PointENUKt {
  @kotlin.OptIn(com.google.protobuf.kotlin.OnlyForUseByGeneratedProtoCode::class)
  @com.google.protobuf.kotlin.ProtoDslMarker
  public class Dsl private constructor(
    private val _builder: apollo.common.Geometry.PointENU.Builder
  ) {
    public companion object {
      @kotlin.jvm.JvmSynthetic
      @kotlin.PublishedApi
      internal fun _create(builder: apollo.common.Geometry.PointENU.Builder): Dsl = Dsl(builder)
    }

    @kotlin.jvm.JvmSynthetic
    @kotlin.PublishedApi
    internal fun _build(): apollo.common.Geometry.PointENU = _builder.build()

    /**
     * ```
     * East from the origin, in meters.
     * ```
     *
     * `optional double x = 1 [default = nan];`
     */
    public var x: kotlin.Double
      @JvmName("getX")
      get() = _builder.getX()
      @JvmName("setX")
      set(value) {
        _builder.setX(value)
      }
    /**
     * ```
     * East from the origin, in meters.
     * ```
     *
     * `optional double x = 1 [default = nan];`
     */
    public fun clearX() {
      _builder.clearX()
    }
    /**
     * ```
     * East from the origin, in meters.
     * ```
     *
     * `optional double x = 1 [default = nan];`
     * @return Whether the x field is set.
     */
    public fun hasX(): kotlin.Boolean {
      return _builder.hasX()
    }

    /**
     * ```
     * North from the origin, in meters.
     * ```
     *
     * `optional double y = 2 [default = nan];`
     */
    public var y: kotlin.Double
      @JvmName("getY")
      get() = _builder.getY()
      @JvmName("setY")
      set(value) {
        _builder.setY(value)
      }
    /**
     * ```
     * North from the origin, in meters.
     * ```
     *
     * `optional double y = 2 [default = nan];`
     */
    public fun clearY() {
      _builder.clearY()
    }
    /**
     * ```
     * North from the origin, in meters.
     * ```
     *
     * `optional double y = 2 [default = nan];`
     * @return Whether the y field is set.
     */
    public fun hasY(): kotlin.Boolean {
      return _builder.hasY()
    }

    /**
     * ```
     * Up from the WGS-84 ellipsoid, in
     * ```
     *
     * `optional double z = 3 [default = 0];`
     */
    public var z: kotlin.Double
      @JvmName("getZ")
      get() = _builder.getZ()
      @JvmName("setZ")
      set(value) {
        _builder.setZ(value)
      }
    /**
     * ```
     * Up from the WGS-84 ellipsoid, in
     * ```
     *
     * `optional double z = 3 [default = 0];`
     */
    public fun clearZ() {
      _builder.clearZ()
    }
    /**
     * ```
     * Up from the WGS-84 ellipsoid, in
     * ```
     *
     * `optional double z = 3 [default = 0];`
     * @return Whether the z field is set.
     */
    public fun hasZ(): kotlin.Boolean {
      return _builder.hasZ()
    }
  }
}
@kotlin.jvm.JvmSynthetic
public inline fun apollo.common.Geometry.PointENU.copy(block: `apollo.common`.PointENUKt.Dsl.() -> kotlin.Unit): apollo.common.Geometry.PointENU =
  `apollo.common`.PointENUKt.Dsl._create(this.toBuilder()).apply { block() }._build()

