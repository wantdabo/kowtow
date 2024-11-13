using Kowtow.Math;

namespace Kowtow.Collision.Shapes
{
    /// <summary>
    /// 圆柱体
    /// </summary>
    public class CylinderShape : Shape
    {
        /// <summary>
        /// 高度
        /// </summary>
        public FP height { get; set; }
        /// <summary>
        /// 半径
        /// </summary>
        public FP radius { get; set; }
        
        /// <summary>
        /// 圆柱体构造函数
        /// </summary>
        /// <param name="center">中心点</param>
        /// <param name="height">高度</param>
        /// <param name="radius">半径</param>
        public CylinderShape(FPVector3 center, FP height, FP radius)
        {
            this.center = center;
            this.height = height;
            this.radius = radius;
        }
        
        public override void SupportMapping(ref FPVector3 direction, out FPVector3 result)
        {
            FP sigma = FP.Sqrt(direction.x * direction.x + direction.z * direction.z);

            if (sigma > FP.Zero)
            {
                result.x = direction.x / sigma * radius;
                result.y = FP.Sign(direction.y) * height * FP.Half;
                result.z = direction.z / sigma * radius;
            }
            else
            {
                result.x = FP.Zero;
                result.y = FP.Sign(direction.y) * height * FP.Half;
                result.z = FP.Zero;
            }
        }
    }
}
