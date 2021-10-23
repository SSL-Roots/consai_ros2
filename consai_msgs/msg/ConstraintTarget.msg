uint32 TARGET_UNKNOWN = 0
uint32 TARGET_BALL = 1
uint32 TARGET_BLUE_ROBOT = 2
uint32 TARGET_YELLOW_ROBOT = 3

uint32 PARAMETER_UNKNOWN = 0
uint32 PARAMETER_X = 1
uint32 PARAMETER_Y = 2
uint32 PARAMETER_THETA = 3
uint32 PARAMETER_LOOK_TO = 4
uint32 PARAMETER_LOOK_FROM = 5

# ロボットやボールが存在しないとき、解が得られない
# そのため、すべての属性をoptionalにしている
float64[<=1] value  # 実数値
uint32[<=1] target  # ボールやロボット等の目標オブジェクト
uint32[<=1] target_id  # ロボットのID
uint32[<=1] target_parameter  # x, y等の目標オブジェクトから参照するパラメータ