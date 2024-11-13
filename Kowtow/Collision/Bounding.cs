using Kowtow.Math;

namespace Kowtow.Collision
{
    /// <summary>
    /// 包围盒
    /// </summary>
    public struct Bounding
    {
        /// <summary>
        /// 中心点
        /// </summary>
        public FPVector3 center { get; set; }
        /// <summary>
        /// 半径
        /// </summary>
        public FP radius { get; set; }
        
        /// <summary>
        /// 检测两个包围盒是否相交
        /// </summary>
        /// <param name="other">其他包围盒</param>
        /// <returns>YES/NO</returns>
        public bool Intersects(Bounding other)
        {
            // 获取两个球心的距离的平方
            FP distanceSquared = (other.center - center).sqrMagnitude; 
            FP radiusSum = radius + other.radius;
            
            // 如果距离小于半径和的平方，则发生碰撞
            return distanceSquared <= radiusSum * radiusSum; 
        }
    }
}
