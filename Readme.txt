
A 改造步骤

1. 制作bittle 的步骤
制作usd格式的模型
通过isaac sim 的urdf import功能（菜单中）把 bittle 的urdf 和 mesh导入 -- 会自动输出为usd格式
注意：
.要将fix_base_link = false ,否则模型不能动
.生成instanceable格式usd模型似乎不行，没有腿，应该是没有符合生成要求把

2. 把bittle.usd 导入到 Nuclues下的usd目录（本地建立的）

3. 增加和修改如下文件(模仿anymal）
增加
- cfg/task/Bittle.yaml （任务的配置）
- cfg/train/BittlePPO.yaml (训练的配置）
- robots/articulations/bittle.py (加载usd，定义关节）
- robots/articulations/views/bittle_view.py (定义base 和 knee 的view，当前用于跌倒判断等）
- task/bittle.py （任务主程序，定义各种训练的基本动作，env，action，obs，rewards等）
修改
- utils/task_utils.py （增加bittle任务）



B 改造中的关键点：
B.1
模型需要存储为usb 时必须设置Fix Base Link = false （否则模型身体不会动），另外udf 似乎存储为instanceable 模式会丢失信息（腿没了）
API 方式应该可以修改 -- https://www.hackster.io/shangxu/tutorial-for-simulating-bittle-in-isaac-sim-bb5b3c

B.2
失败定义 == 如何定义失败呢？ 就是摔倒怎么定义 ——— 身体和四肢跌落到某个水平面下
self.fallen_over = self._bittles.is_base_below_threshold(threshold=0.51, ground_heights=0.0) | self._bittles.is_knee_below_threshold(threshold=0.21, ground_heights=0.0)


B.3
DP 控制是否要修改
force = stiffness * (targetPosition - position) + damping * (targetVelocity - velocity)
The stiffness and damping can be regarded as the P and D term in a typical PID controller. They implies the extent to which the drive attempts to achieve the target position and velocity respectively.
增大比例系数P，将加快系统的响应，在有静差的情况下有利于减小静差，但是过大的比例系数会使系统有较大的超调，并产生振荡，使稳定性变坏。
增大积分时间有利于减小超调，减小振荡，使系统的稳定性增加，但是系统静差消除时间变长。
增大微分时间有利于加快系统的响应速度，使系统超调量小，稳定性增加，但系统对扰动的抑制能力减弱
目前采用anymal的数值
# action scale: target angle = actionScale * action + defaultAngle （参考https://github.com/leggedrobotics/legged_gym/blob/master/legged_gym/envs/a1/a1_config.py）


B.4
关节初始位置（初始位置应该是个能基本站住、我们的站姿和anymal 不同（它是>< 形，我们是>>形）
    left_front_shoulder_joint: 0.4      # [rad] <-
    left_back_shoulder_joint: 1.0       # [rad] <-
    right_front_shoulder_joint: -0.4    # [rad] <-
    right_back_shoulder_joint: -1.0     # [rad] <-
    left_front_knee_joint: -1.0        # [rad] ->
    left_back_knee_joint: -1.2         # [rad] ->
    right_front_knee_joint: 1.0        # [rad] ->
    right_back_knee_joint: 1.2         # [rad] ->

B.5 关节定义顺序问题
self._dof_names = [ #FIX IT  order ?
                    "left_back_shoulder_joint",
                    "left_front_shoulder_joint",
                    "right_back_shoulder_joint",
                    "right_front_shoulder_joint",
                    "left_back_knee_joint",
                    "left_front_knee_joint",
                    "right_back_knee_joint",
                    "right_front_knee_joint"
                           ]
因为要和 dof_pos = self._anymals.get_joint_positions(clone=False) 顺序对上，而后者似乎是usd 文件的内部顺序

B.6 动作快慢问题
  control:
    # PD Drive parameters:
    stiffness: 85.0  # [N*m/rad]
    damping: 4.0     # [N*m*s/rad]
    actionScale: 13.5。— 这个值越大则动作越快 ！！！  10- 300？
  快慢含义还不清楚
  但效果似乎默认值最好

B.7 关节幅度问题 - 减少关节幅度，应该降低了搜索空间，便于学习
joint_limits = {
    "left_back_shoulder_joint":(-40,40),
    "left_front_shoulder_joint":(-40,40),
    "right_back_shoulder_joint":(-40,40),
    "right_front_shoulder_joint":(-40,40),
    "left_back_knee_joint":(-40,0),
    "left_front_knee_joint":(-40,0),
    "right_back_knee_joint":(0,40),
    "right_front_knee_joint":(0,40),
}

B.8 关于摔倒判断，使用角度判断，比高度更合适
  def is_orientation_below_threshold(self, threshold):
        _, base_orientation = self._base.get_world_poses()
        x = base_orientation[:, 1]
        y = base_orientation[:, 2]
        z = base_orientation[:, 3]
        w = base_orientation[:, 0]
        # print("orient:",x,y)
        ang = torch.Tensor(quaternion_to_euler(x, y, z, w))
        # print(a)
        return (ang[0] > threshold) | (ang[0] < -1 * threshold) | (ang[1] > threshold) | (ang[1] < -1 * threshold)

B.9


C 参考资料
anymal社区
https://github.com/ANYbotics/anymal_c_simple_description
知乎介绍
https://www.zhihu.com/column/c_1533191136755236864
训练bittle
https://www.youtube.com/watch?v=d9HEhXH5_hs
模拟bittle
https://www.youtube.com/watch?v=phTnbmXM06g&t=436s
控制bittle
anymal 学习介绍
https://www.youtube.com/watch?v=Afi17BnSuBM
