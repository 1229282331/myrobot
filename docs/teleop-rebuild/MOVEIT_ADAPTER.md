# MoveIt 到现有正式工作台的适配

当前为开发资格版本，默认后端与生产限速未改；完整交付尚未完成。

## 实际调用路径

`phosphobot 原键盘/手柄页面 → useE6Control → PC透明WebSocket代理 → 独立SSH → X5 IntentSession → MoveIt Servo Twist/JointJog → JointTrajectory → 官方SDK ServoJ → E6`

`gateway.py --moveit-bundle <目录>`显式选择备选路线。未提供时仍为原生ServoP/ServoJ，已有失败证据保持。MoveIt重计算在独立ROS节点；Python适配工作线程只做消息转换和KDL审查。E6的30004原始接收、29999命令回包/Stop与网关看门狗均不在该工作线程。

完整phosphobot应用没有换成另一个自写页面。MoveIt没有启动Dobot ROS驱动，不会占用第二个29999连接。影子阶段没有机械臂写入或CAN发送能力。

## 模型与输入对应关系

- 使用`CONTROLLER_MODEL_ALIGNMENT.md`中的控制器等价串联模型、KDL FK/Jacobian和实际MoveIt求解。每个新反馈更新实测关节；启动及运行时分别呈现与控制器Tool0的模型误差。夹爪TCP和碰撞附件不冒充已标定。
- SI米/弧度、xyzw；基座/工具意图交给MoveIt在controller_base/controller_flange之间变换。实际状态取原控制器，不以模型输出覆盖原反馈。
- 上游JointTrajectory的stamp=0表示立即执行，原样保留。独立补丁`patches/moveit/0001-input-provenance.patch`仅在frame_id中附加**本次算法实际选中的输入类型和stamp**，不修改算法和保护。ROS时间只作标识，不与PC/X5单调时钟相减。
- 接收候选时匹配本地发布记录、操作epoch、新鲜度及输入序号，原输入随候选保留。释放清除所有候选；暂停期间原库把平滑器复位到新实测关节。再次操作使用新epoch，旧候选不能重放。
- ROS处理过期或失败导致控制暂停，但不关闭原始反馈、Stop确认或记录已接收的原数据。界面还能看到设备真实状态。正常方向边界保持会话，允许退回。

## 构建与运行资格

固定MoveIt2.5.10源码、官方E6源、生成模型及凸包打包脚本为`teleop-platform/scripts/package_moveit_bundle.py`。X5编译入口为`scripts/build_moveit_bundle.sh`。使用已有ROS Humble依赖，源码包内的微小元数据补丁可独立审查；不覆盖系统APT二进制。

X5可用内存约3GiB，O3并行/单任务及O1受限构建均失败，原日志保留。最终在X5以本机头文件预处理大编译单元，在PC WSL以GCC11.5 ARM64交叉编译，再回X5链接其他对象。实际加载的Servo库路径由进程maps验证；完整编译输入、对象和二进制SHA保留于moveit-adapter-v24。公开prepare/compile/finish脚本已提供，等效底层步骤执行成功，完整包装脚本仍待独立验证。

`scripts/qualify_moveit_bridge.py`在v27完成虚拟36方向和释放检查。正式网页影子v28的590个独立候选、36个方向与来源序号核查通过，82559帧官方SDK读回；运动/Stop/CAN为零。它们都不能替代实际持续速度验收。

正式网页live v29（gain500）与v30（gain750）均完成J6低速小往返及释放，命令确认、Stop回包和静止均可记录，但实际幅度明显不足。v31无设备虚拟实验固定请求0.2°/s：零跟随延迟时实际0.111775°/s，150ms一阶跟随延迟时0.041136°/s。该延迟是声明的测试替身，不声称辨识了E6。此前虚拟方向通过没有证明速度通过；不以新的低速结果宣布交付。

较新上游2.12.4（1ade0e9dcf50dbbbc3a984b995c786cf12736235）的ServoNode已在独立Jazzy环境实际运行：连续轨迹使用有效命令窗口的参考状态，过期回到实测状态。v32无延迟/150ms延迟对照均约0.1985°/s（请求0.2）。默认加速度插件有细小输入死区和SI小约束数值问题，原Ruckig插件对照也未达速。

v34的独立插件保留官方OSQP算法，只修非零增量保留及约束等价缩放/数值精度；使用独立namespace避免覆盖上游安装。7组原插件直接对照及完整ServoNode虚拟闭环见`JAZZY_QUALIFICATION.md`。新版尚未接入正式网页/实机，旧2.5.10桥接仍是历史实现；不能把这次虚拟通过当交付。

目前没有因为代码已写完、模型一致或构建成功而开放最终30 mm/s、10°/s档位。
