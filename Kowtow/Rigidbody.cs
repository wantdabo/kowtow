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
        public Bounding bounding { get; set; }
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
                orientation = FPMatrix.CreateFromQuaternion(value);
            }
        }
        /// <summary>
        /// 旋转矩阵
        /// </summary>
        public FPMatrix orientation { get; private set; }
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
        /// 碰撞关系列表
        /// </summary>
        private List<Collider> colliders = new();

        /// <summary>
        /// 刚体构造函数
        /// </summary>
        /// <param name="shape">几何体</param>
        /// <param name="material">物理材质</param>
        public Rigidbody(Shape shape, Material material)
        {
            this.shape = shape;
            this.material = material;
            orientation = FPMatrix.CreateFromQuaternion(rotation);
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

            velocity += impulse * material.inverseMass;
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
        /// 重置碰撞
        /// </summary>
        public void ResetColliders()
        {
            colliders.Clear();
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
            
            GravityForce();

            // 计算加速
            var acceleration = force * material.inverseMass;
            force = FPVector3.zero;
            // 增加加速
            velocity += acceleration * t;
            // 速度阻尼
            velocity *= (FP.One - drag * t);
            // 更新位置
            position += velocity * t;

            if (0 != colliders.Count)
            {
                foreach (var collider in colliders)
                {
                    // 计算碰撞相对速度
                    FPVector3 relativeVelocity = velocity - collider.rigidbody.velocity;

                    // 碰撞速度沿法线分量
                    FP velocityAlongNormal = FPVector3.Dot(relativeVelocity, collider.normal);

                    // 如果分离状态，不做碰撞响应
                    if (velocityAlongNormal > 0) continue;

                    // 计算反弹速度（考虑材质弹力系数）
                    FP bounciness = FPMath.Min(material.bounciness, collider.rigidbody.material.bounciness);
                    FP impulseMagnitude = -(1 + bounciness) * velocityAlongNormal;
                    impulseMagnitude /= material.inverseMass + collider.rigidbody.material.inverseMass; 

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
