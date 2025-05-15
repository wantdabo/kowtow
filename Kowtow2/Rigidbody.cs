using Kowtow.Collision;
using Kowtow.Collision.Shape;
using Kowtow.Math;

namespace Kowtow
{
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
    public sealed class Rigidbody
    {
        /// <summary>
        /// 刚体 ID
        /// </summary>
        public uint id { get; set; }
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
        /// 上一帧碰撞关系列表
        /// </summary>
        private List<Collider> lastcolliders { get; set; }
        /// <summary>
        /// 碰撞关系列表
        /// </summary>
        private List<Collider> colliders { get; set; }

        public void Ready(uint id)
        {
            this.id = id;
            this.layer = Layer.Default;
            this.type = RigidbodyType.Static;
            this.detection = DetectionType.Discrete;
            this.trigger = false;
            this.position = FPVector3.zero;
            this.rotation = FPQuaternion.identity;
            this.force = FPVector3.zero;
            this.velocity = FPVector3.zero;
            this.drag = FP.Zero;
            this.mass = FP.One;
            this.gravityscale = FP.One;
            this.material = default;
            this.shape = default;
            this.lastcolliders = ObjectPool.Get<List<Collider>>();
            this.colliders = ObjectPool.Get<List<Collider>>();
        }

        public void Reset()
        {
            this.id = 0;
            this.layer = Layer.Default;
            this.type = RigidbodyType.Static;
            this.detection = DetectionType.Discrete;
            this.trigger = false;
            this.position = FPVector3.zero;
            this.rotation = FPQuaternion.identity;
            this.force = FPVector3.zero;
            this.velocity = FPVector3.zero;
            this.drag = FP.Zero;
            this.mass = FP.One;
            this.gravityscale = FP.One;
            this.material = default;
            if (null != this.shape) ObjectPool.Set(this.shape);
            this.shape = default;
            this.lastcolliders.Clear();
            ObjectPool.Set(this.lastcolliders);
            this.colliders.Clear();
            ObjectPool.Set(this.colliders);
        }

        public Rigidbody Clone()
        {
            Rigidbody clone = ObjectPool.Get<Rigidbody>();
            clone.Ready(this.id);
            clone.layer = this.layer;
            clone.type = this.type;
            clone.detection = this.detection;
            clone.trigger = this.trigger;
            clone.position = this.position;
            clone.rotation = this.rotation;
            clone.force = this.force;
            clone.velocity = this.velocity;
            clone.drag = this.drag;
            clone.mass = this.mass;
            clone.gravityscale = this.gravityscale;
            clone.material = this.material;
            if (null != this.shape) clone.shape = this.shape.Clone();
            foreach (var collider in lastcolliders) clone.lastcolliders.Add(collider);
            foreach (var collider in colliders) clone.colliders.Add(collider);
            
            return clone;
        }
    }
}