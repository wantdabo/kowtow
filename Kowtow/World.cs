using Kowtow.Collision;
using Kowtow.Collision.Shapes;
using Kowtow.Math;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Threading.Tasks;

namespace Kowtow
{
    /// <summary>
    /// 世界
    /// </summary>
    public class World
    {
        /// <summary>
        /// 八叉树
        /// </summary>
        public Octree tree { get; private set; }
        /// <summary>
        /// 重力
        /// </summary>
        public FPVector3 gravity { get; set; }
        /// <summary>
        /// 时间间隔
        /// </summary>
        public FP timestep { get; private set; }
        /// <summary>
        /// 刚体列表
        /// </summary>
        private List<Rigidbody> rigidbodies = new();
        /// <summary>
        /// 世界构造函数
        /// </summary>
        /// <param name="gravity">重力</param>
        public World(FPVector3 gravity = default)
        {
            tree = new();
            this.gravity = gravity;
        }

        /// <summary>
        /// 添加刚体
        /// </summary>
        /// <param name="shape">几何体</param>
        /// <param name="mass">质量</param>
        /// <param name="material">物理材质</param>
        /// <returns>刚体</returns>
        public Rigidbody AddRigidbody(Shape shape, FP mass, Material material)
        {
            Rigidbody rigidbody = new(shape, mass, material);
            rigidbody.world = this;
            rigidbodies.Add(rigidbody);
            tree.Rigidbody2Node(rigidbody);

            return rigidbody;
        }

        /// <summary>
        /// 移除刚体
        /// </summary>
        /// <param name="rigidbody">刚体</param>
        public void RmvRigidbody(Rigidbody rigidbody)
        {
            if (false == rigidbodies.Contains(rigidbody)) return;
            tree.RmvRigidbody(rigidbody);
            rigidbody.world = null;
            rigidbodies.Remove(rigidbody);
        }

        /// <summary>
        /// 驱动世界
        /// </summary>
        /// <param name="t">时间间隔</param>
        public void Update(FP t)
        {
            if (FP.Zero == t) return;
            timestep = t;
            
            Detections();
            foreach (var rigidbody in rigidbodies)
            {
                rigidbody.NotifyColliderEvents();
                if (rigidbody.aabbupdated)
                {
                    tree.AABBUpdate(rigidbody);
                    rigidbody.aabbupdated = false;
                }
            }
            Parallel.ForEach(rigidbodies, rigidbody =>
            {
                rigidbody.Update(t);
            });
        }
        
        /// <summary>
        /// 碰撞检测
        /// </summary>
        private void Detections()
        {
            Parallel.ForEach(rigidbodies, self =>
            {
                self.ResetColliders();

                if (false == tree.QueryRigidbodies(self, out var bodies)) return;

                foreach (var target in bodies)
                {
                    if (self == target) continue;
                
                    // 层级检测
                    if (false == Layer.Query(self.layer, target.layer)) continue;
                
                    // 精确碰撞检测
                    if (false == Detection.Detect(self, target, out var point, out var normal, out var penetration)) continue;
                
                    self.AddCollider(new Collider
                    {
                        rigidbody = target,
                        point = point,
                        normal = normal,
                        penetration = penetration
                    });
                }
            });
        }
    }
}
