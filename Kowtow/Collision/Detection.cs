using Kowtow.Collision.Shapes;
using Kowtow.Math;
using System.Collections.Generic;

namespace Kowtow.Collision
{
    /// <summary>
    /// 碰撞检测
    /// </summary>
    public class Detection
    {
        /// <summary>
        /// 检测极限值
        /// </summary>
        private static FP epsilon = FP.EN3;
        /// <summary>
        /// 迭代检测次数
        /// </summary>
        private const int iterations = 5;

        /// <summary>
        /// 刚体与刚体碰撞检测
        /// </summary>
        /// <param name="rigidbody1">刚体 1</param>
        /// <param name="rigidbody2">刚体 2</param>
        /// <param name="point">碰撞点</param>
        /// <param name="normal">从刚体 2 指向刚体 1 的法线</param>
        /// <param name="penetration">穿透深度</param>
        /// <returns>YES/NO</returns>
        public static bool Detect(Rigidbody rigidbody1, Rigidbody rigidbody2, out FPVector3 point, out FPVector3 normal, out FP penetration)
        {
            return Detect(rigidbody1.shape, rigidbody2.shape, rigidbody1.position + rigidbody1.shape.center, rigidbody2.position + rigidbody2.shape.center, rigidbody1.orientation, rigidbody2.orientation, out point, out normal, out penetration);
        }

        /// <summary>
        /// 几何体与几何体碰撞检测
        /// </summary>
        /// <param name="shape1">几何体 1</param>
        /// <param name="shape2">几何体 2</param>
        /// <param name="point">碰撞点</param>
        /// <param name="normal">从几何体 2 指向几何体 1 的法线</param>
        /// <param name="penetration">穿透深度</param>
        /// <returns>YES/NO</returns>
        public static bool Detect(Shape shape1, Shape shape2, FPVector3 position1, FPVector3 position2, FPMatrix orientation1, FPMatrix orientation2, out FPVector3 point, out FPVector3 normal, out FP penetration)
        {
            return DetectBoxBox(shape1 as BoxShape, shape2 as BoxShape, position1, position2, orientation1, orientation2, out point, out normal, out penetration);
        }

        /// <summary>
        /// 检测两个BoxShape之间的碰撞
        /// </summary>
        /// <param name="box1">立方体1</param>
        /// <param name="box2">立方体2</param>
        /// <param name="point">碰撞点</param>
        /// <param name="normal">从立方体2指向立方体1的法线</param>
        /// <param name="penetration">穿透深度</param>
        /// <returns>是否发生碰撞</returns>
        public static bool DetectBoxBox(BoxShape box1, BoxShape box2, FPVector3 position1, FPVector3 position2, FPMatrix orientation1, FPMatrix orientation2, out FPVector3 point, out FPVector3 normal, out FP penetration)
        {
            point = FPVector3.zero;
            normal = FPVector3.zero;
            penetration = FP.MaxValue;

            // 获取两个立方体的轴向向量
            FPVector3[] axes1 = GetAxes(orientation1);
            FPVector3[] axes2 = GetAxes(orientation2);

            // 生成所有分离轴（15个轴）
            List<FPVector3> axes = new List<FPVector3>(axes1);
            axes.AddRange(axes2);
            for (int i = 0; i < axes1.Length; i++)
            {
                for (int j = 0; j < axes2.Length; j++)
                {
                    FPVector3 crossProduct = FPVector3.Cross(axes1[i], axes2[j]);
                    if (crossProduct.sqrMagnitude > FP.Epsilon)
                    {
                        axes.Add(crossProduct.normalized);
                    }
                }
            }

            // 遍历每个轴，检查是否存在分离
            foreach (var axis in axes)
            {
                // 投影两个立方体到当前轴上
                (FP min1, FP max1) = ProjectBoxOntoAxis(box1, position1, orientation1, axis);
                (FP min2, FP max2) = ProjectBoxOntoAxis(box2, position2, orientation2, axis);

                // 计算投影的重叠量
                FP overlap = GetOverlap(min1, max1, min2, max2);

                // 检查是否有分离轴
                if (overlap <= 0)
                {
                    // 存在分离轴，表示无碰撞
                    return false;
                }

                // 如果有碰撞，找到最小重叠量的轴
                if (overlap < penetration)
                {
                    penetration = overlap;
                    normal = axis;
                }
            }

            // 若所有轴都重叠，则发生碰撞
            // 计算碰撞点（将使用穿透最小的轴进行计算）
            point = CalculateCollisionPoint(position1, position2, normal, penetration);

            return true;
        }

        private static FPVector3[] GetAxes(FPMatrix orientation)
        {
            var rotation = FPQuaternion.CreateFromMatrix(orientation);
            // 获取 BoxShape 的局部坐标轴（右、上、前）并返回它们在世界坐标中的方向
            FPVector3 right = rotation * FPVector3.right;
            FPVector3 up = rotation * FPVector3.up;
            FPVector3 forward = rotation * FPVector3.forward;
            return new FPVector3[] { right, up, forward };
        }

        private static (FP min, FP max) ProjectBoxOntoAxis(BoxShape box, FPVector3 position, FPMatrix orientation, FPVector3 axis)
        {
            // 投影立方体的 8 个顶点到轴上，找到投影的最小值和最大值
            FPVector3[] vertices = GetBoxVertices(box, position, orientation);
            FP min = FPVector3.Dot(vertices[0], axis);
            FP max = min;

            for (int i = 1; i < vertices.Length; i++)
            {
                FP projection = FPVector3.Dot(vertices[i], axis);
                min = FP.Min(min, projection);
                max = FP.Max(max, projection);
            }

            return (min, max);
        }

        private static FPVector3[] GetBoxVertices(BoxShape box, FPVector3 position, FPMatrix orientation)
        {
            // 计算立方体的 8 个顶点位置
            FPVector3[] vertices = new FPVector3[8];
            FPVector3 extents = box.size * FP.Half;
            var rotation = FPQuaternion.CreateFromMatrix(orientation);

            // 生成相对于中心的8个顶点
            vertices[0] = position + rotation * new FPVector3(-extents.x, -extents.y, -extents.z);
            vertices[1] = position + rotation * new FPVector3(extents.x, -extents.y, -extents.z);
            vertices[2] = position + rotation * new FPVector3(extents.x, extents.y, -extents.z);
            vertices[3] = position + rotation * new FPVector3(-extents.x, extents.y, -extents.z);
            vertices[4] = position + rotation * new FPVector3(-extents.x, -extents.y, extents.z);
            vertices[5] = position + rotation * new FPVector3(extents.x, -extents.y, extents.z);
            vertices[6] = position + rotation * new FPVector3(extents.x, extents.y, extents.z);
            vertices[7] = position + rotation * new FPVector3(-extents.x, extents.y, extents.z);

            return vertices;
        }

        private static FP GetOverlap(FP min1, FP max1, FP min2, FP max2)
        {
            // 添加 epsilon 来处理精度问题
            FP overlap = FP.Min(max1, max2) - FP.Max(min1, min2);
            return (overlap < epsilon) ? FP.Zero : overlap;
        }

        private static FPVector3 CalculateCollisionPoint(FPVector3 position1, FPVector3 position2, FPVector3 collisionAxis, FP penetration)
        {
            // 计算沿着碰撞法线轴的移动方向
            FPVector3 moveDirection = collisionAxis.normalized;

            // 根据穿透深度计算每个立方体沿着法线轴的移动量
            FP halfPenetration = penetration / 2;

            // 计算碰撞后的调整位置
            FPVector3 newPosition1 = position1 - moveDirection * halfPenetration;
            FPVector3 newPosition2 = position2 + moveDirection * halfPenetration;

            // 返回调整后的碰撞点，避免过度偏移
            return (newPosition1 + newPosition2) / 2;
        }
    }
}
