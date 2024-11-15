using Kowtow;
using Kowtow.Collision;
using Kowtow.Collision.Shapes;
using Kowtow.Math;
using System;
using System.Threading;

// 时间间隔
FP tick = FP.One / 60;
// 毫秒
int ms = (int)(tick * 1000).AsFloat();

// 创建世界，重力 -9.8m/s^2
World world = new World(new FPVector3(0, -981 * FP.EN2, 0));

FP groundArea = 100000;

// 添加地板
var ground = world.AddRigidbody(new BoxShape(FPVector3.zero, new FPVector3(groundArea, 1, groundArea)), FP.One, new Material(FP.One, 3));

// 添加球体
var item = world.AddRigidbody(new SphereShape(FPVector3.zero, FP.Half), FP.One, new Material(FP.One, 3));
item.position = new FPVector3(0, 10, 0);
// 设置为动态
item.type = RigidbodyType.Dynamic;

// 驱动世界
while (true)
{
    world.Update(tick);
    Console.WriteLine($"{item.position}, {item.rotation.eulerAngles}, {item.GetColliders().Count}");
    Thread.Sleep(ms);
}
