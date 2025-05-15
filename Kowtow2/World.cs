using Kowtow.Collision.Shape;
using Kowtow.Math;

namespace Kowtow
{
    /// <summary>
    /// 世界
    /// </summary>
    public sealed class World
    {
        /// <summary>
        /// 刚体自增 ID
        /// </summary>
        private uint increment { get; set; } = 0;
        /// <summary>
        /// 重力
        /// </summary>
        public FPVector3 gravity { get; set; }
        /// <summary>
        /// 时间间隔
        /// </summary>
        public FP timestep { get; private set; }
        /// <summary>
        /// 刚体字典
        /// </summary>
        public Dictionary<uint, Rigidbody> rigidbodydict { get; set; }
        /// <summary>
        /// 刚体列表
        /// </summary>
        public List<Rigidbody> rigidbodies { get; private set; }

        /// <summary>
        /// 世界构造函数
        /// </summary>
        /// <param name="gravity">重力</param>
        public World(FPVector3 gravity)
        {
            this.gravity = gravity;
        }

        public uint AddRigidbody(IShape shape, FP mass, Material material)
        {
            increment++;
            var rigidbody = ObjectPool.Get<Rigidbody>();
            rigidbody.Ready(increment);
            rigidbody.shape = shape;
            rigidbody.mass = mass;
            rigidbody.material = material;
            
            rigidbodydict.Add(increment, rigidbody);
            rigidbodies.Add(rigidbody);
            
            return rigidbody.id;
        }
        
        public void RemoveRigidbody(uint id)
        {
            if (false == rigidbodydict.TryGetValue(id, out var rigidbody)) return;
            rigidbody.Reset();
            rigidbodydict.Remove(id);
            rigidbodies.Remove(rigidbody);
        }

        public Box CreateBox(FPVector3 center, FPVector3 size)
        {
            var box = ObjectPool.Get<Box>();
            box.center = center;
            box.size = size;
            
            return box;
        }
        
        public Sphere CreateSphere(FPVector3 center, FP radius)
        {
            var sphere = ObjectPool.Get<Sphere>();
            sphere.center = center;
            sphere.radius = radius;
            
            return sphere;
        }
    }
}