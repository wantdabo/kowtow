using Kowtow.Collision.Shapes;
using Kowtow.Math;

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
        /// 映射
        /// </summary>
        /// <param name="shape">几何体</param>
        /// <param name="orientation">旋转矩阵</param>
        /// <param name="position">位置</param>
        /// <param name="direction">方向</param>
        /// <param name="result">结果</param>
        private static void Mapping(Shape shape, ref FPMatrix orientation, ref FPVector3 position, ref FPVector3 direction, out FPVector3 result)
        {
            result.x = ((direction.x * orientation.M11) + (direction.y * orientation.M12)) + (direction.z * orientation.M13);
            result.y = ((direction.x * orientation.M21) + (direction.y * orientation.M22)) + (direction.z * orientation.M23);
            result.z = ((direction.x * orientation.M31) + (direction.y * orientation.M32)) + (direction.z * orientation.M33);

            shape.SupportMapping(ref result, out result);

            FP x = ((result.x * orientation.M11) + (result.y * orientation.M21)) + (result.z * orientation.M31);
            FP y = ((result.x * orientation.M12) + (result.y * orientation.M22)) + (result.z * orientation.M32);
            FP z = ((result.x * orientation.M13) + (result.y * orientation.M23)) + (result.z * orientation.M33);

            result.x = position.x + x;
            result.y = position.y + y;
            result.z = position.z + z;
        }
        
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
            // Used variables
            FPVector3 temp1, temp2;
            FPVector3 v01, v02, v0;
            FPVector3 v11, v12, v1;
            FPVector3 v21, v22, v2;
            FPVector3 v31, v32, v3;
            FPVector3 v41 = FPVector3.zero, v42 = FPVector3.zero, v4 = FPVector3.zero;
            FPVector3 mn;

            // Initialization of the output
            point = normal = FPVector3.zero;
            penetration = FP.Zero;

            //JVector right = JVector.Right;

            // Get the center of shape1 in world coordinates -> v01
            shape1.SupportCenter(out v01);
            FPVector3.Transform(ref v01, ref orientation1, out v01);
            FPVector3.Add(ref position1, ref v01, out v01);

            // Get the center of shape2 in world coordinates -> v02
            shape2.SupportCenter(out v02);
            FPVector3.Transform(ref v02, ref orientation2, out v02);
            FPVector3.Add(ref position2, ref v02, out v02);

            // v0 is the center of the minkowski difference
            FPVector3.Subtract(ref v02, ref v01, out v0);

            // Avoid case where centers overlap -- any direction is fine in this case
            if (v0.IsNearlyZero()) v0 = new FPVector3(FP.EN4, 0, 0);

            // v1 = support in direction of origin
            mn = v0;
            FPVector3.Negate(ref v0, out normal);

            Mapping(shape1, ref orientation1, ref position1, ref mn, out v11);
            Mapping(shape2, ref orientation2, ref position2, ref normal, out v12);
            FPVector3.Subtract(ref v12, ref v11, out v1);

            if (FPVector3.Dot(ref v1, ref normal) <= FP.Zero) return false;

            // v2 = support perpendicular to v1,v0
            FPVector3.Cross(ref v1, ref v0, out normal);

            if (normal.IsNearlyZero())
            {
                FPVector3.Subtract(ref v1, ref v0, out normal);

                normal.Normalize();

                point = v11;
                FPVector3.Add(ref point, ref v12, out point);
                FPVector3.Multiply(ref point, FP.Half, out point);

                FPVector3.Subtract(ref v12, ref v11, out temp1);
                penetration = FPVector3.Dot(ref temp1, ref normal);

                //point = v11;
                //point2 = v12;
                return true;
            }

            FPVector3.Negate(ref normal, out mn);
            Mapping(shape1, ref orientation1, ref position1, ref mn, out v21);
            Mapping(shape2, ref orientation2, ref position2, ref normal, out v22);
            FPVector3.Subtract(ref v22, ref v21, out v2);

            if (FPVector3.Dot(ref v2, ref normal) <= FP.Zero) return false;

            // Determine whether origin is on + or - side of plane (v1,v0,v2)
            FPVector3.Subtract(ref v1, ref v0, out temp1);
            FPVector3.Subtract(ref v2, ref v0, out temp2);
            FPVector3.Cross(ref temp1, ref temp2, out normal);

            FP dist = FPVector3.Dot(ref normal, ref v0);

            // If the origin is on the - side of the plane, reverse the direction of the plane
            if (dist > FP.Zero)
            {
                FPVector3.Swap(ref v1, ref v2);
                FPVector3.Swap(ref v11, ref v21);
                FPVector3.Swap(ref v12, ref v22);
                FPVector3.Negate(ref normal, out normal);
            }


            int phase2 = 0;
            int phase1 = 0;
            bool hit = false;

            // Phase One: Identify a portal
            while (true)
            {
                if (phase1 > iterations) return false;

                phase1++;

                // Obtain the support point in a direction perpendicular to the existing plane
                // Note: This point is guaranteed to lie off the plane
                FPVector3.Negate(ref normal, out mn);
                Mapping(shape1, ref orientation1, ref position1, ref mn, out v31);
                Mapping(shape2, ref orientation2, ref position2, ref normal, out v32);
                FPVector3.Subtract(ref v32, ref v31, out v3);


                if (FPVector3.Dot(ref v3, ref normal) <= FP.Zero)
                {
                    return false;
                }

                // If origin is outside (v1,v0,v3), then eliminate v2 and loop
                FPVector3.Cross(ref v1, ref v3, out temp1);
                if (FPVector3.Dot(ref temp1, ref v0) < FP.Zero)
                {
                    v2 = v3;
                    v21 = v31;
                    v22 = v32;
                    FPVector3.Subtract(ref v1, ref v0, out temp1);
                    FPVector3.Subtract(ref v3, ref v0, out temp2);
                    FPVector3.Cross(ref temp1, ref temp2, out normal);
                    continue;
                }

                // If origin is outside (v3,v0,v2), then eliminate v1 and loop
                FPVector3.Cross(ref v3, ref v2, out temp1);
                if (FPVector3.Dot(ref temp1, ref v0) < FP.Zero)
                {
                    v1 = v3;
                    v11 = v31;
                    v12 = v32;
                    FPVector3.Subtract(ref v3, ref v0, out temp1);
                    FPVector3.Subtract(ref v2, ref v0, out temp2);
                    FPVector3.Cross(ref temp1, ref temp2, out normal);
                    continue;
                }

                // Phase Two: Refine the portal
                // We are now inside of a wedge...
                while (true)
                {
                    phase2++;

                    // Compute normal of the wedge face
                    FPVector3.Subtract(ref v2, ref v1, out temp1);
                    FPVector3.Subtract(ref v3, ref v1, out temp2);
                    FPVector3.Cross(ref temp1, ref temp2, out normal);
                    // Beginer

                    // Can this happen???  Can it be handled more cleanly?
                    if (normal.IsNearlyZero()) return true;

                    normal.Normalize();
                    // Compute distance from origin to wedge face
                    FP d = FPVector3.Dot(ref normal, ref v1);


                    // If the origin is inside the wedge, we have a hit
                    if (d >= 0 && !hit)
                    {
                        // HIT!!!
                        hit = true;
                    }

                    // Find the support point in the direction of the wedge face
                    FPVector3.Negate(ref normal, out mn);
                    Mapping(shape1, ref orientation1, ref position1, ref mn, out v41);
                    Mapping(shape2, ref orientation2, ref position2, ref normal, out v42);
                    FPVector3.Subtract(ref v42, ref v41, out v4);

                    FPVector3.Subtract(ref v4, ref v3, out temp1);
                    FP delta = FPVector3.Dot(ref temp1, ref normal);
                    penetration = FPVector3.Dot(ref v4, ref normal);

                    // If the boundary is thin enough or the origin is outside the support plane for the newly discovered vertex, then we can terminate
                    if (delta <= epsilon || penetration <= FP.Zero || phase2 > iterations)
                    {

                        if (hit)
                        {
                            FPVector3.Cross(ref v1, ref v2, out temp1);
                            FP b0 = FPVector3.Dot(ref temp1, ref v3);
                            FPVector3.Cross(ref v3, ref v2, out temp1);
                            FP b1 = FPVector3.Dot(ref temp1, ref v0);
                            FPVector3.Cross(ref v0, ref v1, out temp1);
                            FP b2 = FPVector3.Dot(ref temp1, ref v3);
                            FPVector3.Cross(ref v2, ref v1, out temp1);
                            FP b3 = FPVector3.Dot(ref temp1, ref v0);

                            FP sum = b0 + b1 + b2 + b3;

                            if (sum <= 0)
                            {
                                b0 = 0;
                                FPVector3.Cross(ref v2, ref v3, out temp1);
                                b1 = FPVector3.Dot(ref temp1, ref normal);
                                FPVector3.Cross(ref v3, ref v1, out temp1);
                                b2 = FPVector3.Dot(ref temp1, ref normal);
                                FPVector3.Cross(ref v1, ref v2, out temp1);
                                b3 = FPVector3.Dot(ref temp1, ref normal);

                                sum = b1 + b2 + b3;
                            }

                            FP inv = FP.One / sum;

                            FPVector3.Multiply(ref v01, b0, out point);
                            FPVector3.Multiply(ref v11, b1, out temp1);
                            FPVector3.Add(ref point, ref temp1, out point);
                            FPVector3.Multiply(ref v21, b2, out temp1);
                            FPVector3.Add(ref point, ref temp1, out point);
                            FPVector3.Multiply(ref v31, b3, out temp1);
                            FPVector3.Add(ref point, ref temp1, out point);

                            FPVector3.Multiply(ref v02, b0, out temp2);
                            FPVector3.Add(ref temp2, ref point, out point);
                            FPVector3.Multiply(ref v12, b1, out temp1);
                            FPVector3.Add(ref point, ref temp1, out point);
                            FPVector3.Multiply(ref v22, b2, out temp1);
                            FPVector3.Add(ref point, ref temp1, out point);
                            FPVector3.Multiply(ref v32, b3, out temp1);
                            FPVector3.Add(ref point, ref temp1, out point);

                            FPVector3.Multiply(ref point, inv * FP.Half, out point);

                        }

                        return hit;
                    }

                    FPVector3.Cross(ref v4, ref v0, out temp1);

                    FP dot = FPVector3.Dot(ref temp1, ref v1);

                    if (dot >= FP.Zero)
                    {
                        dot = FPVector3.Dot(ref temp1, ref v2);

                        if (dot >= FP.Zero)
                        {
                            // Inside d1 & inside d2 ==> eliminate v1
                            v1 = v4;
                            v11 = v41;
                            v12 = v42;
                        }
                        else
                        {
                            // Inside d1 & outside d2 ==> eliminate v3
                            v3 = v4;
                            v31 = v41;
                            v32 = v42;
                        }
                    }
                    else
                    {
                        dot = FPVector3.Dot(ref temp1, ref v3);

                        if (dot >= FP.Zero)
                        {
                            // Outside d1 & inside d3 ==> eliminate v2
                            v2 = v4;
                            v21 = v41;
                            v22 = v42;
                        }
                        else
                        {
                            // Outside d1 & outside d3 ==> eliminate v1
                            v1 = v4;
                            v11 = v41;
                            v12 = v42;
                        }
                    }
                }
            }
        }
    }
}
