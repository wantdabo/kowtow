using Kowtow.Math;

namespace Kowtow.Collision.Shapes
{
    /// <summary>
    /// 立方体
    /// </summary>
    public class BoxShape : Shape
    {
        /// <summary>
        /// 尺寸
        /// </summary>
        public FPVector3 size { get; set; }

        /// <summary>
        /// 立方体构造函数
        /// </summary>
        /// <param name="center">中心点</param>
        /// <param name="size">尺寸</param>
        public BoxShape(FPVector3 center, FPVector3 size)
        {
            this.center = center;
            this.size = size;
        }

        public override void SupportMapping(ref FPVector3 direction, out FPVector3 result)
        {
            var halfsize = size * FP.Half;
            result.x = FP.Sign(direction.x) * halfsize.x;
            result.y = FP.Sign(direction.y) * halfsize.y;
            result.z = FP.Sign(direction.z) * halfsize.z;
        }
    }
}
