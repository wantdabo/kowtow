using Kowtow.Math;

namespace Kowtow.Collision.Shape
{
    /// <summary>
    /// 几何体
    /// </summary>
    public interface IShape
    {
        /// <summary>
        /// 几何体类型
        /// </summary>
        public ShapeDef type { get; }
        /// <summary>
        /// 中心点
        /// </summary>
        public FPVector3 center { get; set; }
        /// <summary>
        /// 克隆
        /// </summary>
        /// <returns>几何体</returns>
        public IShape Clone();
    }
}