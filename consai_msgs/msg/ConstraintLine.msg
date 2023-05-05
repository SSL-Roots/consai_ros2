# 直線による拘束

# P1とP2を結ぶ直線Line1to2を作る
ConstraintXY p1
ConstraintXY p2
# Line1の直線上で、P1からdistanceだけ離れた位置を目標位置とする
float64 distance
# 姿勢theta
ConstraintTheta theta

# P3とP4をセットした場合、P3とP4を結ぶ直線Line3to4を作り
# Line1to2上で、Line3to4と交わるところを目標位置とする
ConstraintXY[<=1] p3
ConstraintXY[<=1] p4 
# Line1to2上での交点からp2へのオフセット距離
# 負の値を入れるとp1側へオフセットする
# このオフセット距離によって目標位置がp1, p2をはみ
float64[<=1] offset_intersection_to_p2
