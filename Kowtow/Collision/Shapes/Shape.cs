using Kowtow.Math;

namespace Kowtow.Collision.Shapes
{
    /// <summary>
    /// 几何体
    /// </summary>
    public abstract class Shape
    {
        /// <summary>
        /// 中心点
        /// </summary>
        public FPVector3 center { get; set; }

        /// <summary>
        /// 获取中心点
        /// </summary>
        /// <param name="center">中心点</param>
        public void SupportCenter(out FPVector3 center)
        {
            center = this.center;
        }
    }
}
