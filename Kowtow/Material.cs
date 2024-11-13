using Kowtow.Math;

namespace Kowtow
{
    /// <summary>
    /// 物理材质
    /// </summary>
    public class Material
    {
        /// <summary>
        /// 质量
        /// </summary>
        public FP mass { get; set; }
        /// <summary>
        /// One Div Mass (1 / mass)
        /// </summary>
        public FP inverseMass { get { return FP.One / mass; } }
        /// <summary>
        /// 摩檫力
        /// </summary>
        public FP friction { get; set; }
        /// <summary>
        /// 弹性
        /// </summary>
        public FP bounciness { get; set; }
        
        /// <summary>
        /// 物理材质构造函数
        /// </summary>
        /// <param name="mass">质量</param>
        /// <param name="friction">摩檫力</param>
        /// <param name="bounciness">弹力</param>
        public Material(FP mass, FP friction, FP bounciness)
        {
            this.mass = mass;
            this.friction = friction;
            this.bounciness = bounciness;
        }
    }
}
