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
        /// 中心点
        /// </summary>
        public FPVector3 center { get; set; }
        /// <summary>
        /// 包围盒
        /// </summary>
        public AABB aabb { get; set; }
        /// <summary>
        /// 半径
        /// </summary>
        public FP radius { get; set; }
        
        /// <summary>
        /// 球体构造函数
        /// </summary>
        /// <param name="center">中心点</param>
        /// <param name="radius">半径</param>
        public Sphere(FPVector3 center, FP radius)
        {
            this.center = center;
            this.radius = radius;
        }
    }
}