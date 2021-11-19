# 直線による拘束

# P1とP2を結ぶ直線Line1to2を作る
ConstraintXY p1
ConstraintXY p2
# Line1の直線上で、P1からdistanceだけ離れた位置を目標位置とする
float64 distance
# 姿勢theta
ConstraintTheta theta

# P3とP4をセットした場合、P3とP4を結ぶ直線Line3to4を作り
# Line1to2とLine3to4の交点を目標位置とする
ConstraintXY[<=1] p3
ConstraintXY[<=1] p4 