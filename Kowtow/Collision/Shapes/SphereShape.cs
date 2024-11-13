using Kowtow.Math;

namespace Kowtow.Collision.Shapes
{
    /// <summary>
    /// 球体
    /// </summary>
    public class SphereShape : Shape
    {
        /// <summary>
        /// 半径
        /// </summary>
        public FP radius { get; set; }
        
        /// <summary>
        /// 球体构造函数
        /// </summary>
        /// <param name="center">中心点</param>
        /// <param name="radius">半径</param>
        public SphereShape(FPVector3 center, FP radius)
        {
            this.center = center;
            this.radius = radius;
        }
        
        public override void SupportMapping(ref FPVector3 direction, out FPVector3 result)
        {
            result = direction;
            result.Normalize();
            
            FPVector3.Multiply(ref result, radius, out result);
        }
    }
}
