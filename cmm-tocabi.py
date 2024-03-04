import pinocchio as pin
import numpy as np

urdf_path = "./robots/dyros_tocabi.urdf"
model, collision_model, visual_model = pin.buildModelsFromUrdf(urdf_path)
data = model.createData()

q = pin.randomConfiguration(model)
print("model.nv: ", model.nv)

qdot = pin.utils.zero(model.nv)
qdot_t = pin.utils.zero(model.nv)
qddot_t = pin.utils.zero(model.nv)

for j in range(0, 33):
    qdot[j] = np.random.uniform(-0.1, 0.1)
    qdot_t[j] = 0.0
    qddot_t[j] = 0.0

# CMM = pin.computeCentroidalMap(model, data, q)
# print("CMM :\n", CMM)
# print(len(q), len(qdot))
# q = np.matrix(q).reshape(-1, 1)
# qdot = np.matrix(qdot).reshape(-1, 1)
# pin.computeCentroidalMap(model, data, q, qdot)
pin.computeCentroidalMomentum(model, data, q, qdot)
pin.crba(model, data, q)
pin.computeCoriolisMatrix(model, data, q, qdot)
pin.rnea(model, data, q, qdot_t, qddot_t)

print("data.C :\n", data.C.flatten().T)
print("data.M :\n", data.M.flatten().T)
print("data.tau :\n", data.tau.flatten().T)

h = data.hg  # 중심 각운동량 벡터 (3x1 벡터)
Ag = data.Ag  # 중심 각운동량 행렬



print("h.angular: ", h.angular)
roll_momentum = h.angular[0]  # Roll 방향 각운동량
pitch_momentum = h.angular[1]  # Pitch 방향 각운동량
yaw_momentum = h.angular[2]  # Yaw 방향 각운동량

# 결과 출력
print("Roll Angular Momentum:", roll_momentum)
print("Pitch Angular Momentum:", pitch_momentum)
print("Yaw Angular Momentum:", yaw_momentum)
# 결과 출력
# print("Centroidal Angular Momentum Vector:", h.flatten().T)
# print("Centroidal Angular Momentum Matrix:\n", Ag)

# CMM_slice = CMM[:,6:39]
# COR_slice = data.C[:,6:39]
# M_slice = data.M[:,6:39]
# g_slice = data.tau[6:39]


# print("CMM_slice :\n", CMM_slice)
# print("COR_slice :\n", COR_slice)
# print("M_slice :\n", M_slice)
# print("g_slice :\n", g_slice)