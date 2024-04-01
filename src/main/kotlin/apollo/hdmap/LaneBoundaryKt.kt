// Generated by the protocol buffer compiler. DO NOT EDIT!
// source: modules/map/proto/map_lane.proto

// Generated files should ignore deprecation warnings
@file:Suppress("DEPRECATION")
package apollo.hdmap;

@kotlin.jvm.JvmName("-initializelaneBoundary")
public inline fun laneBoundary(block: apollo.hdmap.LaneBoundaryKt.Dsl.() -> kotlin.Unit): apollo.hdmap.MapLane.LaneBoundary =
  apollo.hdmap.LaneBoundaryKt.Dsl._create(apollo.hdmap.MapLane.LaneBoundary.newBuilder()).apply { block() }._build()
/**
 * Protobuf type `apollo.hdmap.LaneBoundary`
 */
public object LaneBoundaryKt {
  @kotlin.OptIn(com.google.protobuf.kotlin.OnlyForUseByGeneratedProtoCode::class)
  @com.google.protobuf.kotlin.ProtoDslMarker
  public class Dsl private constructor(
    private val _builder: apollo.hdmap.MapLane.LaneBoundary.Builder
  ) {
    public companion object {
      @kotlin.jvm.JvmSynthetic
      @kotlin.PublishedApi
      internal fun _create(builder: apollo.hdmap.MapLane.LaneBoundary.Builder): Dsl = Dsl(builder)
    }

    @kotlin.jvm.JvmSynthetic
    @kotlin.PublishedApi
    internal fun _build(): apollo.hdmap.MapLane.LaneBoundary = _builder.build()

    /**
     * `optional .apollo.hdmap.Curve curve = 1;`
     */
    public var curve: apollo.hdmap.MapGeometry.Curve
      @JvmName("getCurve")
      get() = _builder.getCurve()
      @JvmName("setCurve")
      set(value) {
        _builder.setCurve(value)
      }
    /**
     * `optional .apollo.hdmap.Curve curve = 1;`
     */
    public fun clearCurve() {
      _builder.clearCurve()
    }
    /**
     * `optional .apollo.hdmap.Curve curve = 1;`
     * @return Whether the curve field is set.
     */
    public fun hasCurve(): kotlin.Boolean {
      return _builder.hasCurve()
    }
    public val LaneBoundaryKt.Dsl.curveOrNull: apollo.hdmap.MapGeometry.Curve?
      get() = _builder.curveOrNull

    /**
     * `optional double length = 2;`
     */
    public var length: kotlin.Double
      @JvmName("getLength")
      get() = _builder.getLength()
      @JvmName("setLength")
      set(value) {
        _builder.setLength(value)
      }
    /**
     * `optional double length = 2;`
     */
    public fun clearLength() {
      _builder.clearLength()
    }
    /**
     * `optional double length = 2;`
     * @return Whether the length field is set.
     */
    public fun hasLength(): kotlin.Boolean {
      return _builder.hasLength()
    }

    /**
     * ```
     * indicate whether the lane boundary exists in real world
     * ```
     *
     * `optional bool virtual = 3;`
     */
    public var virtual: kotlin.Boolean
      @JvmName("getVirtual")
      get() = _builder.getVirtual()
      @JvmName("setVirtual")
      set(value) {
        _builder.setVirtual(value)
      }
    /**
     * ```
     * indicate whether the lane boundary exists in real world
     * ```
     *
     * `optional bool virtual = 3;`
     */
    public fun clearVirtual() {
      _builder.clearVirtual()
    }
    /**
     * ```
     * indicate whether the lane boundary exists in real world
     * ```
     *
     * `optional bool virtual = 3;`
     * @return Whether the virtual field is set.
     */
    public fun hasVirtual(): kotlin.Boolean {
      return _builder.hasVirtual()
    }

    /**
     * An uninstantiable, behaviorless type to represent the field in
     * generics.
     */
    @kotlin.OptIn(com.google.protobuf.kotlin.OnlyForUseByGeneratedProtoCode::class)
    public class BoundaryTypeProxy private constructor() : com.google.protobuf.kotlin.DslProxy()
    /**
     * ```
     * in ascending order of s
     * ```
     *
     * `repeated .apollo.hdmap.LaneBoundaryType boundary_type = 4;`
     */
     public val boundaryType: com.google.protobuf.kotlin.DslList<apollo.hdmap.MapLane.LaneBoundaryType, BoundaryTypeProxy>
      @kotlin.jvm.JvmSynthetic
      get() = com.google.protobuf.kotlin.DslList(
        _builder.getBoundaryTypeList()
      )
    /**
     * ```
     * in ascending order of s
     * ```
     *
     * `repeated .apollo.hdmap.LaneBoundaryType boundary_type = 4;`
     * @param value The boundaryType to add.
     */
    @kotlin.jvm.JvmSynthetic
    @kotlin.jvm.JvmName("addBoundaryType")
    public fun com.google.protobuf.kotlin.DslList<apollo.hdmap.MapLane.LaneBoundaryType, BoundaryTypeProxy>.add(value: apollo.hdmap.MapLane.LaneBoundaryType) {
      _builder.addBoundaryType(value)
    }
    /**
     * ```
     * in ascending order of s
     * ```
     *
     * `repeated .apollo.hdmap.LaneBoundaryType boundary_type = 4;`
     * @param value The boundaryType to add.
     */
    @kotlin.jvm.JvmSynthetic
    @kotlin.jvm.JvmName("plusAssignBoundaryType")
    @Suppress("NOTHING_TO_INLINE")
    public inline operator fun com.google.protobuf.kotlin.DslList<apollo.hdmap.MapLane.LaneBoundaryType, BoundaryTypeProxy>.plusAssign(value: apollo.hdmap.MapLane.LaneBoundaryType) {
      add(value)
    }
    /**
     * ```
     * in ascending order of s
     * ```
     *
     * `repeated .apollo.hdmap.LaneBoundaryType boundary_type = 4;`
     * @param values The boundaryType to add.
     */
    @kotlin.jvm.JvmSynthetic
    @kotlin.jvm.JvmName("addAllBoundaryType")
    public fun com.google.protobuf.kotlin.DslList<apollo.hdmap.MapLane.LaneBoundaryType, BoundaryTypeProxy>.addAll(values: kotlin.collections.Iterable<apollo.hdmap.MapLane.LaneBoundaryType>) {
      _builder.addAllBoundaryType(values)
    }
    /**
     * ```
     * in ascending order of s
     * ```
     *
     * `repeated .apollo.hdmap.LaneBoundaryType boundary_type = 4;`
     * @param values The boundaryType to add.
     */
    @kotlin.jvm.JvmSynthetic
    @kotlin.jvm.JvmName("plusAssignAllBoundaryType")
    @Suppress("NOTHING_TO_INLINE")
    public inline operator fun com.google.protobuf.kotlin.DslList<apollo.hdmap.MapLane.LaneBoundaryType, BoundaryTypeProxy>.plusAssign(values: kotlin.collections.Iterable<apollo.hdmap.MapLane.LaneBoundaryType>) {
      addAll(values)
    }
    /**
     * ```
     * in ascending order of s
     * ```
     *
     * `repeated .apollo.hdmap.LaneBoundaryType boundary_type = 4;`
     * @param index The index to set the value at.
     * @param value The boundaryType to set.
     */
    @kotlin.jvm.JvmSynthetic
    @kotlin.jvm.JvmName("setBoundaryType")
    public operator fun com.google.protobuf.kotlin.DslList<apollo.hdmap.MapLane.LaneBoundaryType, BoundaryTypeProxy>.set(index: kotlin.Int, value: apollo.hdmap.MapLane.LaneBoundaryType) {
      _builder.setBoundaryType(index, value)
    }
    /**
     * ```
     * in ascending order of s
     * ```
     *
     * `repeated .apollo.hdmap.LaneBoundaryType boundary_type = 4;`
     */
    @kotlin.jvm.JvmSynthetic
    @kotlin.jvm.JvmName("clearBoundaryType")
    public fun com.google.protobuf.kotlin.DslList<apollo.hdmap.MapLane.LaneBoundaryType, BoundaryTypeProxy>.clear() {
      _builder.clearBoundaryType()
    }

  }
}
@kotlin.jvm.JvmSynthetic
public inline fun apollo.hdmap.MapLane.LaneBoundary.copy(block: `apollo.hdmap`.LaneBoundaryKt.Dsl.() -> kotlin.Unit): apollo.hdmap.MapLane.LaneBoundary =
  `apollo.hdmap`.LaneBoundaryKt.Dsl._create(this.toBuilder()).apply { block() }._build()

public val apollo.hdmap.MapLane.LaneBoundaryOrBuilder.curveOrNull: apollo.hdmap.MapGeometry.Curve?
  get() = if (hasCurve()) getCurve() else null

