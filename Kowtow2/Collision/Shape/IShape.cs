using Kowtow.Math;

namespace Kowtow.Collision.Shape
{
    /// <summary>
    /// 几何体
    /// </summary>
    public interface IShape
    {
        /// <summary>
        /// 中心点
        /// </summary>
        public FPVector3 center { get; set; }
        /// <summary>
        /// 包围盒
        /// </summary>
        public AABB aabb { get; set; }
    }
}