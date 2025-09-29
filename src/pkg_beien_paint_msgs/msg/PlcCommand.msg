# 消息头 包含：时间戳、帧ID
std_msgs/Header header
# 指令来源/ID，便于追踪调试
string source_id
# 寄存器起始地址
uint16 start_address
# 连续寄存器数据
uint16[] register_values
# 写入优先级
uint8 priority
# 操作类型：0=写，1=清零，2=复位
uint8 op_type
# 写入模式：0=覆盖写，1=变化写，2=强制写
uint8 write_mode
# 指令过期时间（相对或绝对）
builtin_interfaces/Duration expire_duration