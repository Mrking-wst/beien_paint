# 左手柄按钮
bool zl
bool l
bool minus      # “-” 键
bool function   # "⭕"功能键

bool up
bool down
bool left
bool right
bool up_right
bool down_right
bool down_left
bool up_left
bool sr
bool sl

# 左摇杆
uint16 stick_x   # 0 ~ 4095, 中点2048
uint16 stick_y   # 0 ~ 4095, 中点2047

# 原始数据
uint8[12] raw_data  