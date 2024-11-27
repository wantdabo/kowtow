# Kowtow
这是基于 .NET 开发的 3D 轻量物理引擎，支持确定性计算与正常浮点运算

### 大致全貌
- 重力、力、冲量
- 八叉空间树、离散碰撞、连续碰撞
- 立方体、球体、射线、线段
- 定点数运算/浮点数运算【宏】、多线程 (不影响确定性计算)【宏】

#### [项目结构](#projectdire)
### <span id="catalog">目录</span>
- [1.快速开始](#qstart)
- [2.环境配置](#installenv)
  - [1.安装 .NET](#installenv.1)
  - [2.在 Unity 工作](#installenv.2)

---
### TODO
- 八叉树迭代修改为 Stack (现, 递归)
- 角速度
- Mesh 几何体
---

#### <span id="qstart">1.快速开始</span>
- 1.开发环境中，需要安装 [**.NET 2.0+**](#installenv.1)
- 2.使用 Rider/VisualStudio/CustomIDE 打开 **Kowtow.sln**，运行 **Kowtow.Test** 项目即可
- 3.构造物理世界
    ```csharp
        // 创建世界，重力 -9.8m/s^2
        World world = new World(new FPVector3(0, -981 * FP.EN2, 0));

        // 添加立方体地板
        var ground = world.AddRigidbody(new BoxShape(FPVector3.zero, new FPVector3(100000, 1, 100000)), FP.One, new Material(FP.One, FP.Zero));

        // 添加球体
        var ball = world.AddRigidbody(new SphereShape(FPVector3.zero, FP.Half), FP.One, new Material(FP.One, 0));
        ball.position = new FPVector3(0, 10, 0);
        // 设置为动态
        ball.type = RigidbodyType.Dynamic;
        // 设置为连续碰撞检测
        ball.detection = DetectionType.Continuous;

        // 设定帧率为 10
        FP tick = FP.One / 10;
        // 驱动世界
        while (true)
        {
            world.Update(tick);
            Console.WriteLine($"position -> {ball.position}, velocity -> {ball.velocity}, collider -> {ball.GetColliders().Count}");
            Thread.Sleep(tick.AsFloat() * 1000);
        }
     ```
#### <span id="installenv">2.环境配置</span>
- ##### <span id="installenv.1">1.安装 .NET</span>
  - 该项目，Luban 配置工具依赖，需要安装 [**.NET 2.0+**](https://dotnet.microsoft.com/zh-cn/download)
- ##### <span id="installenv.2">2.在 Unity 工作</span>
  - 复制 **Kowtow/Kowtow/** 到 Unity 项目中即可/编译 DLL 到 Unity 项目中即可
---

#### <span id="projectdire">项目结构</span>
```text
└─Kowtow
    ├─Kowtow
    │   ├─Collision
    │   └─Math
    └─Kowtow.Test
```
- **Kowtow/Kowtow/** 物理核心
- **Kowtow/Kowtow/Collision/** 碰撞检测算法、几何体定义、八叉树
- **Kowtow/Kowtow/Math/** 数学库
- **Kowtow/Kowtow.Test/** 测试用例