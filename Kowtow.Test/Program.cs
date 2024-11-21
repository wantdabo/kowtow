using Kowtow;
using Kowtow.Collision;
using Kowtow.Collision.Shapes;
using Kowtow.Math;
using System;
using System.Diagnostics;
using System.Threading;

// Stopwatch sw = Stopwatch.StartNew();
// sw.Reset();
// sw.Restart();
// for (int i = 0; i < 1000000; i++)
// {
//     var muldiv = i * 10 / 100;
//     var addsub = i + 10 - 100;
//     var sin = FPMath.Sin(90);
//     var cos = FPMath.Cos(90);
// }
// sw.Stop();
// Console.WriteLine($"Elapsed -> {sw.ElapsedMilliseconds}");
//
// Console.ReadKey();

// 时间间隔
FP tick = FP.One / 120;
// 毫秒
int ms = (int)(tick * 1000).AsFloat();

// 创建世界，重力 -9.8m/s^2
World world = new World(new FPVector3(0, -981 * FP.EN2, 0));

FP groundArea = 100000;

// 添加地板
var ground = world.AddRigidbody(new BoxShape(FPVector3.zero, new FPVector3(groundArea, 1, groundArea)), FP.One, new Material(FP.One, 0));

// 添加球体
var ball = world.AddRigidbody(new SphereShape(FPVector3.zero, FP.Half), FP.One, new Material(FP.One, 0));
ball.position = new FPVector3(-10, 10, -10);
// 设置为动态
ball.type = RigidbodyType.Dynamic;
// 设置为连续碰撞检测
ball.detection = DetectionType.Continuous;

// FPRandom random = FPRandom.New(19491001);
// for (int i = 0; i < 1000; i++)
// {
//     var ball2 = world.AddRigidbody(new SphereShape(FPVector3.zero, FP.Half), FP.One, new Material(FP.One, 3));
//     ball2.position = new FPVector3(random.Next(-50000, 50000), 10, random.Next(-50000, 50000));
//     ball2.type = RigidbodyType.Dynamic;
//     ball2.detection = DetectionType.Continuous;
// }

var sw = Stopwatch.StartNew();
// 驱动世界
while (true)
{
    sw.Reset();
    sw.Start();
    world.Update(tick);
    sw.Stop();
    Console.Title = $"ms: {sw.ElapsedMilliseconds}";
    
    Console.WriteLine($"{ball.position}, {ball.rotation.eulerAngles}, {ball.GetColliders().Count}");
    Thread.Sleep(ms);
}
