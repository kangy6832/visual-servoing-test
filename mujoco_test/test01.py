import mujoco 
import os

model_path = os.path.expanduser("~/mujoco/model/humanoid/humanoid.xml")

# MjModel 代表完整的物理模型
# from_xml_path 用于从指定路径的XML文件构建MjModel对象
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)
print(data)

# 调用计时API确保其他组件正常安装
for _ in range(1000): mujoco.mj_step(model, data)

print("Test done.")