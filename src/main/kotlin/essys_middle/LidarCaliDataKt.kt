// Generated by the protocol buffer compiler. DO NOT EDIT!
// source: modules/essys/lidar_cali.proto

// Generated files should ignore deprecation warnings
@file:Suppress("DEPRECATION")
package essys_middle;

@kotlin.jvm.JvmName("-initializelidarCaliData")
public inline fun lidarCaliData(block: essys_middle.LidarCaliDataKt.Dsl.() -> kotlin.Unit): essys_middle.LidarCali.LidarCaliData =
  essys_middle.LidarCaliDataKt.Dsl._create(essys_middle.LidarCali.LidarCaliData.newBuilder()).apply { block() }._build()
/**
 * Protobuf type `essys_middle.LidarCaliData`
 */
public object LidarCaliDataKt {
  @kotlin.OptIn(com.google.protobuf.kotlin.OnlyForUseByGeneratedProtoCode::class)
  @com.google.protobuf.kotlin.ProtoDslMarker
  public class Dsl private constructor(
    private val _builder: essys_middle.LidarCali.LidarCaliData.Builder
  ) {
    public companion object {
      @kotlin.jvm.JvmSynthetic
      @kotlin.PublishedApi
      internal fun _create(builder: essys_middle.LidarCali.LidarCaliData.Builder): Dsl = Dsl(builder)
    }

    @kotlin.jvm.JvmSynthetic
    @kotlin.PublishedApi
    internal fun _build(): essys_middle.LidarCali.LidarCaliData = _builder.build()

    /**
     * An uninstantiable, behaviorless type to represent the field in
     * generics.
     */
    @kotlin.OptIn(com.google.protobuf.kotlin.OnlyForUseByGeneratedProtoCode::class)
    public class TfDataProxy private constructor() : com.google.protobuf.kotlin.DslProxy()
    /**
     * `repeated .essys_middle.LidarTransformData tf_data = 1;`
     */
     public val tfData: com.google.protobuf.kotlin.DslList<essys_middle.LidarCali.LidarTransformData, TfDataProxy>
      @kotlin.jvm.JvmSynthetic
      get() = com.google.protobuf.kotlin.DslList(
        _builder.getTfDataList()
      )
    /**
     * `repeated .essys_middle.LidarTransformData tf_data = 1;`
     * @param value The tfData to add.
     */
    @kotlin.jvm.JvmSynthetic
    @kotlin.jvm.JvmName("addTfData")
    public fun com.google.protobuf.kotlin.DslList<essys_middle.LidarCali.LidarTransformData, TfDataProxy>.add(value: essys_middle.LidarCali.LidarTransformData) {
      _builder.addTfData(value)
    }
    /**
     * `repeated .essys_middle.LidarTransformData tf_data = 1;`
     * @param value The tfData to add.
     */
    @kotlin.jvm.JvmSynthetic
    @kotlin.jvm.JvmName("plusAssignTfData")
    @Suppress("NOTHING_TO_INLINE")
    public inline operator fun com.google.protobuf.kotlin.DslList<essys_middle.LidarCali.LidarTransformData, TfDataProxy>.plusAssign(value: essys_middle.LidarCali.LidarTransformData) {
      add(value)
    }
    /**
     * `repeated .essys_middle.LidarTransformData tf_data = 1;`
     * @param values The tfData to add.
     */
    @kotlin.jvm.JvmSynthetic
    @kotlin.jvm.JvmName("addAllTfData")
    public fun com.google.protobuf.kotlin.DslList<essys_middle.LidarCali.LidarTransformData, TfDataProxy>.addAll(values: kotlin.collections.Iterable<essys_middle.LidarCali.LidarTransformData>) {
      _builder.addAllTfData(values)
    }
    /**
     * `repeated .essys_middle.LidarTransformData tf_data = 1;`
     * @param values The tfData to add.
     */
    @kotlin.jvm.JvmSynthetic
    @kotlin.jvm.JvmName("plusAssignAllTfData")
    @Suppress("NOTHING_TO_INLINE")
    public inline operator fun com.google.protobuf.kotlin.DslList<essys_middle.LidarCali.LidarTransformData, TfDataProxy>.plusAssign(values: kotlin.collections.Iterable<essys_middle.LidarCali.LidarTransformData>) {
      addAll(values)
    }
    /**
     * `repeated .essys_middle.LidarTransformData tf_data = 1;`
     * @param index The index to set the value at.
     * @param value The tfData to set.
     */
    @kotlin.jvm.JvmSynthetic
    @kotlin.jvm.JvmName("setTfData")
    public operator fun com.google.protobuf.kotlin.DslList<essys_middle.LidarCali.LidarTransformData, TfDataProxy>.set(index: kotlin.Int, value: essys_middle.LidarCali.LidarTransformData) {
      _builder.setTfData(index, value)
    }
    /**
     * `repeated .essys_middle.LidarTransformData tf_data = 1;`
     */
    @kotlin.jvm.JvmSynthetic
    @kotlin.jvm.JvmName("clearTfData")
    public fun com.google.protobuf.kotlin.DslList<essys_middle.LidarCali.LidarTransformData, TfDataProxy>.clear() {
      _builder.clearTfData()
    }

  }
}
@kotlin.jvm.JvmSynthetic
public inline fun essys_middle.LidarCali.LidarCaliData.copy(block: `essys_middle`.LidarCaliDataKt.Dsl.() -> kotlin.Unit): essys_middle.LidarCali.LidarCaliData =
  `essys_middle`.LidarCaliDataKt.Dsl._create(this.toBuilder()).apply { block() }._build()

