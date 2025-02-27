## PCL 处理设计思路

### 总体思路

将 ROS 2 话题接收与点云处理分离，并根据数据流向进一步拆分点云处理功能，如过滤点云、提取平面、点云可视化等。

完整的处理流程如下：

1. 程序启动时读取 `config.json` 并保存到全局 `json` 类。
2. 接收到 ROS 2 数据时进入回调函数，从消息中提取点云数据。
3. 复制 `json` 配置，将其与点云数据传入处理管线。
4. 处理完成后通过 ROS 2 发布结果。

```cpp
void subscriptionCallback(const PointCloud2Msg_t::SharedPtr msg)
{
    std::shared_ptr<PCL_Handle::PointCloud_t> PointCloud_ptr(new PCL_Handle::PointCloud_t);
    json temp_content = _content;
    pcl::fromROSMsg(*msg, *PointCloud_ptr);
    if (PointCloud_ptr->empty()) return;

    _PclPipeLine.handlePCL(PointCloud_ptr, temp_content);

    PointCloud2Msg_t msg_out;
    if (get_parameter("pub_xyzitr").as_bool()) {
        auto PointCloud_ptr_xyzitr = pointToPointXYZITR(PointCloud_ptr);
        pcl::toROSMsg(*PointCloud_ptr_xyzitr, msg_out);
    } else {
        pcl::toROSMsg(*PointCloud_ptr, msg_out);
    }
    msg_out.header = msg->header;
    _Publisher_ptr->publish(msg_out);
}
```

下图为完整的框图
![](../../.github/整体流程图.png)
### 设计说明

- 点云处理的核心在 `_PclPipeLine.handlePCL(PointCloud_ptr, temp_content)`，其余部分负责 ROS 2 消息处理。
- `PclFactory` 类中的 `_PclPipeLine` 包含三个具体的处理类，数据依次经过这些类的流水线处理。
- 每个类都实现了相同的 `handlePCL` 接口，接收点云数据与 `json` 配置，输出保存到 `json` 或全局共享内存。
- `PclFactory` 还维护着一个 `unordered_map`，作为全局共享内存，用于保存不易通过 `json` 传输的信息。

### 数据流框图

### PclFactory 设计

1. **PclFactory** 类中的 `_PclPipeLine` 包含了三个具体的处理类，数据在进入 `_PclPipeLine` 后依次经过这三个类进行流水线处理。
2. 每个类都实现了相同的 `handlePCL` 接口，接收点云原始数据和 `json` 配置，输出可保存到 `json` 或全局共享内存。

```cpp
void PclFactory_t::handlePCL(std::shared_ptr<PointCloud_t> &PointInput_ptr, json &content)
{
    for (auto &pclHandle : _PclPipeLine) {
        pclHandle->handlePCL(PointInput_ptr, content);
    }
}
```

3. 通过管线遍历所有类，依次调用 `handlePCL` 方法。
4. 配置文件实时读取，不需要重新编译即可修改参数或重新组合节点。

按照通常的写法是你给上面三个功能各自写一套函数,然后是类似下面的结构
```c++
function1(...);
function2(...);
function3(...);
```
但是上述开发各个代码之间的依赖性很强,如果我让你去掉第一个功能或者在第一二个功能之间加一个新的功能你很容易写g掉。
### 设计优点

- **解耦合**：消除了各功能模块间的强依赖性。
- **动态可配置**：可以在不停止程序的情况下实时调整参数。

### 设计挑战

1. **上下文管理**：
   - 每段代码依赖的参数不同，因此通过 `json` 传递参数。
   - 通过 `json["pcl_filter"]["min_z"]` 访问参数。
2. **共享内存管理**：
   - 通过 `unordered_map` 存储复杂数据结构，避免 `json` 的限制。

```cpp
auto normal_ptr_name = content["output"]["normal"].get<std::string>();
if (_global_ptr->find(normal_ptr_name) != _global_ptr->end()) {
    auto normal_ptr = std::any_cast<pcl::PointCloud<pcl::Normal>::Ptr>(_global_ptr->at(normal_ptr_name));
    _viewer_ptr->removePointCloud("normals");
    _viewer_ptr->addPointCloudNormals<Point_t, pcl::Normal>(PointShow_ptr, normal_ptr, 1, 0.1, "normals");
}
```

### 其他问题与解答

1. **为什么回调中重新赋值 `json`？**
   - 防止旧数据残留影响处理。
2. **如何实现多类型点云数据？**
   - 使用 C++ 虚函数与类的继承机制。[虚函数介绍](https://blog.csdn.net/qq_42864343/article/details/134488942?utm_medium=distribute.pc_relevant.none-task-blog-2~default~baidujs_baidulandingword~default-12-134488942-blog-131458144.235^v43^pc_blog_bottom_relevance_base1&spm=1001.2101.3001.4242.7&utm_relevant_index=14)
3. **如何处理多次节点调用？**
   - 通过 `searchParam` 函数实现参数区分。
4. **为什么使用指针引用？**
   - 传递引用以便直接修改原数据。
5. **如何动态创建管线节点？**
   - 通过 `PclFactory_t::createHandle()` 根据 `json` 读取的字符串动态创建节点。
6. **如何添加新节点？**
   - 在 `pcl_implenmention.hpp` 中定义新类，继承 `PclInterface_t`，重写 `handlePCL` 和 `getName` 方法，在 `createHandle()` 中注册。

### JSON 示例

```json
{
    "pcl_pipe_line": [
        "PclFilterZero",
        "PclSegmentation",
        "PclVisual",
        "PclCenterTransform"
    ],
    "PclFilterZero": {
        "default": {
            "min_z": 0.5,
            "max_z": 3.2,
            "max_r": 6,
            "zero_z": 0.181
        }
    }
}
```

- `pcl_pipe_line` 确定了处理管线的节点。
- `PclFilterZero` 保存了该节点的默认配置。
- 运行时的输出保存在程序内的 `json` 类 `output` 中，不会写入文件。

