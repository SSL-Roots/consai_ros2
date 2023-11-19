
string layer "default"
string sub_layer "default"
uint32 z_order 0  # 数字が大きいほど手前に表示される

# ウィンドウ領域に描画するオブジェクト
ShapeAnnotation[] annotations

# フィールド領域に描画するオブジェクト
ShapePoint[] points
ShapeLine[] lines
ShapeArc[] arcs
ShapeRectangle[] rects
ShapeCircle[] circles 
ShapeTube[] tubes
ShapeRobot[] robots
