using Kowtow.Collision;
using Kowtow.Collision.Shapes;
using Kowtow.Math;
using System;
using System.Collections.Generic;

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
    /// 碰撞关系
    /// </summary>
    public struct Collider
    {
        /// <summary>
        /// 刚体
        /// </summary>
        public Rigidbody rigidbody { get; set; }
        /// <summary>
        /// 碰撞点
        /// </summary>
        public FPVector3 point { get; set; }
        /// <summary>
        /// 相对法线
        /// </summary>
        public FPVector3 normal { get; set; }
        /// <summary>
        /// 穿透深度
        /// </summary>
        public FP penetration { get; set; }
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
        /// 世界
        /// </summary>
        public World world { get; set; }
        /// <summary>
        /// 触发器 [开启后不会发生碰撞，但会触发事件]
        /// </summary>
        public bool trigger { get; set; }
        /// <summary>
        /// 几何体
        /// </summary>
        public Shape shape { get; set; }
        /// <summary>
        /// 包围盒
        /// </summary>
        public AABB aabb { get; set; }
        /// <summary>
        /// 质量
        /// </summary>
        public FP mass { get; set; } = FP.One;
        /// <summary>
        /// One Div Mass (1 / mass)
        /// </summary>
        public FP inverseMass { get { return FP.One / mass; } }
        /// <summary>
        /// 物理材质
        /// </summary>
        public Material material { get; set; }
        /// <summary>
        /// 重力缩放
        /// </summary>
        public FP gravityScale { get; set; } = FP.One;
        /// <summary>
        /// 位置
        /// </summary>
        public FPVector3 position { get; set; }
        private FPQuaternion mrotation { get; set; }
        /// <summary>
        /// 旋转
        /// </summary>
        public FPQuaternion rotation
        {
            get { return mrotation; }
            set
            {
                mrotation = value;
            }
        }
        /// <summary>
        /// 力
        /// </summary>
        public FPVector3 force { get; private set; }
        /// <summary>
        /// 速度阻力
        /// </summary>
        public FP drag { get; set; } = FP.Zero;
        /// <summary>
        /// 速度
        /// </summary>
        public FPVector3 velocity { get; set; }
        /// <summary>
        /// 上一帧碰撞关系列表
        /// </summary>
        private List<Collider> lastcolliders = new();
        /// <summary>
        /// 碰撞关系列表
        /// </summary>
        private List<Collider> colliders = new();

        /// <summary>
        /// Collision 进入事件
        /// </summary>
        public event Action<Collider> CollisionEnter;
        /// <summary>
        /// Collision 持续事件
        /// </summary>
        public event Action<Collider> CollisionStay;
        /// <summary>
        /// Collision 离开事件
        /// </summary>
        public event Action<Collider> CollisionExit;
        /// <summary>
        /// Trigger 进入事件
        /// </summary>
        public event Action<Collider> TriggerEnter;
        /// <summary>
        /// Trigger 持续事件
        /// </summary>
        public event Action<Collider> TriggerStay;
        /// <summary>
        /// Trigger 离开事件
        /// </summary>
        public event Action<Collider> TriggerExit;

        /// <summary>
        /// 刚体构造函数
        /// </summary>
        /// <param name="shape">几何体</param>
        /// <param name="mass">质量</param>
        /// <param name="material">物理材质</param>
        public Rigidbody(Shape shape, FP mass, Material material)
        {
            this.shape = shape;
            this.mass = mass;
            this.material = material;
        }

        /// <summary>
        /// 施加力
        /// </summary>
        /// <param name="f">力</param>
        public void ApplyForce(FPVector3 f)
        {
            if (RigidbodyType.Static == type) return;

            force += f;
        }

        /// <summary>
        /// 施加冲量
        /// </summary>
        /// <param name="impulse">冲量</param>
        public void ApplyImpulse(FPVector3 impulse)
        {
            if (RigidbodyType.Static == type) return;

            velocity += impulse * inverseMass;
        }

        /// <summary>
        /// 获取碰撞列表
        /// </summary>
        /// <returns>碰撞列表</returns>
        public List<Collider> GetColliders()
        {
            return colliders;
        }

        /// <summary>
        /// 添加碰撞
        /// </summary>
        /// <param name="collider">碰撞关系</param>
        public void AddCollider(Collider collider)
        {
            colliders.Add(collider);
        }

        /// <summary>
        /// 重置碰撞信息
        /// </summary>
        public void ResetColliders()
        {
            lastcolliders.Clear();
            lastcolliders.AddRange(colliders);
            colliders.Clear();
        }

        /// <summary>
        /// 通知碰撞事件
        /// </summary>
        public void NotifyColliderEvents()
        {
            foreach (var collider in colliders)
            {
                if (false == lastcolliders.Contains(collider))
                {
                    if (trigger)
                    {
                        TriggerEnter?.Invoke(collider);
                    }
                    else
                    {
                        CollisionEnter?.Invoke(collider);
                    }
                }
                else
                {
                    if (trigger)
                    {
                        TriggerStay?.Invoke(collider);
                    }
                    else
                    {
                        CollisionStay?.Invoke(collider);
                    }
                }
            }

            foreach (var collider in lastcolliders)
            {
                if (false == colliders.Contains(collider))
                {
                    if (trigger)
                    {
                        TriggerExit?.Invoke(collider);
                    }
                    else
                    {
                        CollisionExit?.Invoke(collider);
                    }
                }
            }
        }

        /// <summary>
        /// 计算重力
        /// </summary>
        private void GravityForce()
        {
            var gt = world.gravity * gravityScale;
            ApplyForce(gt);
        }

        /// <summary>
        /// 驱动刚体
        /// </summary>
        /// <param name="t">时间间隔</param>
        public void Update(FP t)
        {
            if (RigidbodyType.Static == type) return;
            
            // 计算重力
            GravityForce();

            // 计算加速
            var acceleration = force * inverseMass;
            force = FPVector3.zero;
            // 增加加速
            velocity += acceleration * t;
            // 速度阻尼
            velocity *= (FP.One - drag * t);
            // 更新位置
            position += velocity * t;

            if (trigger) return;
            
            // 计算碰撞接触产生的作用力
            if (0 != colliders.Count)
            {
                foreach (var collider in colliders)
                {
                    if (collider.rigidbody.trigger) continue;
                    
                    // 计算碰撞相对速度
                    FPVector3 relativeVelocity = velocity - collider.rigidbody.velocity;

                    // 碰撞速度沿法线分量
                    FP velocityAlongNormal = FPVector3.Dot(relativeVelocity, collider.normal);

                    // 如果分离状态，不做碰撞响应
                    if (velocityAlongNormal > 0) continue;

                    // 计算反弹速度（考虑材质弹力系数）
                    FP bounciness = FPMath.Min(material.bounciness, collider.rigidbody.material.bounciness);
                    FP impulseMagnitude = -(1 + bounciness) * velocityAlongNormal;
                    impulseMagnitude /= inverseMass + collider.rigidbody.inverseMass;

                    // 应用冲量
                    FPVector3 impulse = impulseMagnitude * collider.normal;
                    ApplyImpulse(impulse);

                    // 计算摩擦力
                    FPVector3 tangent = relativeVelocity - collider.normal * velocityAlongNormal;
                    FP frictionMagnitude = tangent.magnitude * material.friction * collider.rigidbody.material.friction;

                    if (frictionMagnitude > 0)
                    {
                        // 摩擦力方向与切向速度方向相反
                        FPVector3 frictionForce = -tangent.normalized * frictionMagnitude;
                        ApplyForce(frictionForce);
                    }

                    // 穿透修正
                    FPVector3 correction = collider.normal * collider.penetration;
                    position += correction;
                }
            }
        }
    }
}
