using Kowtow.Collision;
using Kowtow.Collision.Shape;
using Kowtow.Math;

namespace Kowtow;

/// <summary>
/// 刚体类型
/// </summary>
public enum RigidbodyType
{
    /// <summary>
    /// 动态
    /// </summary>
    Dynamic,
    /// <summary>
    /// 静态
    /// </summary>
    Static,
}

/// <summary>
/// 碰撞检测类型
/// </summary>
public enum DetectionType
{
    /// <summary>
    /// 离散的
    /// </summary>
    Discrete,
    /// <summary>
    /// 连续的
    /// </summary>
    Continuous,
}
    
/// <summary>
/// 刚体
/// </summary>
public class Rigidbody
{
    /// <summary>
    /// 物理层
    /// </summary>
    public int layer { get; set; } = Layer.Default;
    /// <summary>
    /// 刚体类型
    /// </summary>
    public RigidbodyType type { get; set; } = RigidbodyType.Static;
    /// <summary>
    /// 碰撞检测类型
    /// </summary>
    public DetectionType detection { get; set; } = DetectionType.Discrete;
    /// <summary>
    /// 触发器 [开启后不会发生碰撞，但会触发事件]
    /// </summary>
    public bool trigger { get; set; }
    /// <summary>
    /// 位置
    /// </summary>
    public FPVector3 position { get; set; }
    /// <summary>
    /// 旋转
    /// </summary>
    public FPQuaternion rotation { get; set; }
    /// <summary>
    /// 力
    /// </summary>
    public FPVector3 force { get; set; }
    /// <summary>
    /// 速度
    /// </summary>
    public FPVector3 velocity { get; set; }
    /// <summary>
    /// 空气阻力
    /// </summary>
    public FP drag { get; set; } = FP.Zero;
    /// <summary>
    /// 质量
    /// </summary>
    public FP mass { get; set; }
    /// <summary>
    /// One Div Mass (1 / mass)
    /// </summary>
    public FP inversemass { get => FP.One / mass; }
    /// <summary>
    /// 重力缩放
    /// </summary>
    public FP gravityscale { get; set; }
    /// <summary>
    /// 物理材质
    /// </summary>
    public Material material { get; set; }
    /// <summary>
    /// 几何体
    /// </summary>
    public IShape shape { get; set; }
    /// <summary>
    /// AABB 包围盒
    /// </summary>
    public AABB aabb { get; set; }
    /// <summary>
    /// 上一帧碰撞关系列表
    /// </summary>
    private List<Collider> lastcolliders = new();
    /// <summary>
    /// 碰撞关系列表
    /// </summary>
    private List<Collider> colliders = new();
}