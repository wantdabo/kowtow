using Kowtow.Collision.Shape;
using Kowtow.Math;

namespace Kowtow.Collision.Shape
{
    /// <summary>
    /// 球体
    /// </summary>
    public class Sphere : IShape
    {
        /// <summary>
        /// 球体
        /// </summary>
        public ShapeDef type => ShapeDef.Box;
        /// <summary>
        /// 中心点
        /// </summary>
        public FPVector3 center { get; set; }
        /// <summary>
        /// 半径
        /// </summary>
        public FP radius { get; set; }
        
        /// <summary>
        /// 克隆
        /// </summary>
        /// <returns>几何体</returns>
        public IShape Clone()
        {
            var clone = ObjectPool.Get<Sphere>();
            clone.center = center;
            clone.radius = radius;
            
            return clone;
        }
    }
}