using Kowtow.Collision.Shape;
using Kowtow.Math;

namespace Kowtow.Collision.Shape
{
    /// <summary>
    /// 立方体
    /// </summary>
    public class Box : IShape
    {
        /// <summary>
        /// 立方体
        /// </summary>
        public ShapeDef type => ShapeDef.Box;
        /// <summary>
        /// 中心点
        /// </summary>
        public FPVector3 center { get; set; }
        /// <summary>
        /// 尺寸
        /// </summary>
        public FPVector3 size { get; set; }

        /// <summary>
        /// 克隆
        /// </summary>
        /// <returns>几何体</returns>
        public IShape Clone()
        {
            var clone = ObjectPool.Get<Box>();
            clone.center = center;
            clone.size = size;
            
            return clone;
        }
    }
}