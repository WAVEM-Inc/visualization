// Generated by the protocol buffer compiler. DO NOT EDIT!
// source: modules/common/proto/pnc_point.proto

// Generated files should ignore deprecation warnings
@file:Suppress("DEPRECATION")
package apollo.common;

@kotlin.jvm.JvmName("-initializetrajectory")
public inline fun trajectory(block: apollo.common.TrajectoryKt.Dsl.() -> kotlin.Unit): apollo.common.PncPoint.Trajectory =
  apollo.common.TrajectoryKt.Dsl._create(apollo.common.PncPoint.Trajectory.newBuilder()).apply { block() }._build()
/**
 * Protobuf type `apollo.common.Trajectory`
 */
public object TrajectoryKt {
  @kotlin.OptIn(com.google.protobuf.kotlin.OnlyForUseByGeneratedProtoCode::class)
  @com.google.protobuf.kotlin.ProtoDslMarker
  public class Dsl private constructor(
    private val _builder: apollo.common.PncPoint.Trajectory.Builder
  ) {
    public companion object {
      @kotlin.jvm.JvmSynthetic
      @kotlin.PublishedApi
      internal fun _create(builder: apollo.common.PncPoint.Trajectory.Builder): Dsl = Dsl(builder)
    }

    @kotlin.jvm.JvmSynthetic
    @kotlin.PublishedApi
    internal fun _build(): apollo.common.PncPoint.Trajectory = _builder.build()

    /**
     * `optional string name = 1;`
     */
    public var name: kotlin.String
      @JvmName("getName")
      get() = _builder.getName()
      @JvmName("setName")
      set(value) {
        _builder.setName(value)
      }
    /**
     * `optional string name = 1;`
     */
    public fun clearName() {
      _builder.clearName()
    }
    /**
     * `optional string name = 1;`
     * @return Whether the name field is set.
     */
    public fun hasName(): kotlin.Boolean {
      return _builder.hasName()
    }

    /**
     * An uninstantiable, behaviorless type to represent the field in
     * generics.
     */
    @kotlin.OptIn(com.google.protobuf.kotlin.OnlyForUseByGeneratedProtoCode::class)
    public class TrajectoryPointProxy private constructor() : com.google.protobuf.kotlin.DslProxy()
    /**
     * `repeated .apollo.common.TrajectoryPoint trajectory_point = 2;`
     */
     public val trajectoryPoint: com.google.protobuf.kotlin.DslList<apollo.common.PncPoint.TrajectoryPoint, TrajectoryPointProxy>
      @kotlin.jvm.JvmSynthetic
      get() = com.google.protobuf.kotlin.DslList(
        _builder.getTrajectoryPointList()
      )
    /**
     * `repeated .apollo.common.TrajectoryPoint trajectory_point = 2;`
     * @param value The trajectoryPoint to add.
     */
    @kotlin.jvm.JvmSynthetic
    @kotlin.jvm.JvmName("addTrajectoryPoint")
    public fun com.google.protobuf.kotlin.DslList<apollo.common.PncPoint.TrajectoryPoint, TrajectoryPointProxy>.add(value: apollo.common.PncPoint.TrajectoryPoint) {
      _builder.addTrajectoryPoint(value)
    }
    /**
     * `repeated .apollo.common.TrajectoryPoint trajectory_point = 2;`
     * @param value The trajectoryPoint to add.
     */
    @kotlin.jvm.JvmSynthetic
    @kotlin.jvm.JvmName("plusAssignTrajectoryPoint")
    @Suppress("NOTHING_TO_INLINE")
    public inline operator fun com.google.protobuf.kotlin.DslList<apollo.common.PncPoint.TrajectoryPoint, TrajectoryPointProxy>.plusAssign(value: apollo.common.PncPoint.TrajectoryPoint) {
      add(value)
    }
    /**
     * `repeated .apollo.common.TrajectoryPoint trajectory_point = 2;`
     * @param values The trajectoryPoint to add.
     */
    @kotlin.jvm.JvmSynthetic
    @kotlin.jvm.JvmName("addAllTrajectoryPoint")
    public fun com.google.protobuf.kotlin.DslList<apollo.common.PncPoint.TrajectoryPoint, TrajectoryPointProxy>.addAll(values: kotlin.collections.Iterable<apollo.common.PncPoint.TrajectoryPoint>) {
      _builder.addAllTrajectoryPoint(values)
    }
    /**
     * `repeated .apollo.common.TrajectoryPoint trajectory_point = 2;`
     * @param values The trajectoryPoint to add.
     */
    @kotlin.jvm.JvmSynthetic
    @kotlin.jvm.JvmName("plusAssignAllTrajectoryPoint")
    @Suppress("NOTHING_TO_INLINE")
    public inline operator fun com.google.protobuf.kotlin.DslList<apollo.common.PncPoint.TrajectoryPoint, TrajectoryPointProxy>.plusAssign(values: kotlin.collections.Iterable<apollo.common.PncPoint.TrajectoryPoint>) {
      addAll(values)
    }
    /**
     * `repeated .apollo.common.TrajectoryPoint trajectory_point = 2;`
     * @param index The index to set the value at.
     * @param value The trajectoryPoint to set.
     */
    @kotlin.jvm.JvmSynthetic
    @kotlin.jvm.JvmName("setTrajectoryPoint")
    public operator fun com.google.protobuf.kotlin.DslList<apollo.common.PncPoint.TrajectoryPoint, TrajectoryPointProxy>.set(index: kotlin.Int, value: apollo.common.PncPoint.TrajectoryPoint) {
      _builder.setTrajectoryPoint(index, value)
    }
    /**
     * `repeated .apollo.common.TrajectoryPoint trajectory_point = 2;`
     */
    @kotlin.jvm.JvmSynthetic
    @kotlin.jvm.JvmName("clearTrajectoryPoint")
    public fun com.google.protobuf.kotlin.DslList<apollo.common.PncPoint.TrajectoryPoint, TrajectoryPointProxy>.clear() {
      _builder.clearTrajectoryPoint()
    }

  }
}
@kotlin.jvm.JvmSynthetic
public inline fun apollo.common.PncPoint.Trajectory.copy(block: `apollo.common`.TrajectoryKt.Dsl.() -> kotlin.Unit): apollo.common.PncPoint.Trajectory =
  `apollo.common`.TrajectoryKt.Dsl._create(this.toBuilder()).apply { block() }._build()

