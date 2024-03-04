import pinocchio as pin
# from pinocchio import RobotWrapper

# 로봇 모델 로드 (URDF 파일 경로 필요)
# urdf_path = "/home/hokyun20/RL_cc_ws/src/dyros_tocabi_v2/tocabi_description/robots/dyros_tocabi_0714.urdf"
urdf_path = "./robots/dyros_tocabi.urdf"
# model, collision_model, visual_model = pin.buildModelsFromUrdf(urdf_path, pin.JointModelFreeFlyer())
model, collision_model, visual_model = pin.buildModelsFromUrdf(urdf_path)
data = model.createData()

# 로봇의 현재 상태 설정 (예시)
q = pin.neutral(model)  # 관절 위치
v = pin.utils.zero(model.nv)  # 관절 속도
a = pin.utils.zero(model.nv)  # 관절 가속도

# 중심 각운동량 행렬 계산
pin.computeCentroidalMap(model, data, q, v)
h = data.hg  # 중심 각운동량 벡터 (3x1 벡터)
Ag = data.Ag  # 중심 각운동량 행렬

# 결과 출력
print("Centroidal Angular Momentum Vector:", h.T)
print("Centroidal Angular Momentum Matrix:\n", Ag)