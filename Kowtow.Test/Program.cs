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
World world = new World(new FPVector3(0, -918 * FP.EN2, 0));

FP groundArea = 100;

// 添加地板
var ground = world.AddRigidbody(new BoxShape(FPVector3.zero, new FPVector3(groundArea, 1, groundArea)), new Material(FP.One, FP.One, FP.Zero));

// 添加立方体
var cube = world.AddRigidbody(new BoxShape(FPVector3.zero, FPVector3.one), new Material(FP.One, FP.One, FP.Zero));
cube.rotation = FPQuaternion.Euler(new FPVector3(50, 0, 0));
cube.position = new FPVector3(0, 10, 0);

// 设置为动态
cube.type = RigidbodyType.Dynamic;

for (int i = 0; i < 10; i++)
{
    var c = world.AddRigidbody(new BoxShape(FPVector3.zero, FPVector3.one), new Material(FP.One, FP.One, FP.Zero));
    c.position = new FPVector3((i + 1) * 3, 10, 0);
    // 设置为动态
    c.type = RigidbodyType.Dynamic;
}

// 驱动世界
while (true)
{
    world.Update(tick);
    Console.WriteLine($"{cube.position}, {cube.rotation.eulerAngles}, {cube.GetColliders().Count}");
    Thread.Sleep(ms);
}
