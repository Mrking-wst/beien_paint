#ifndef PLC_SCHEDULER_HPP
#define PLC_SCHEDULER_HPP

#include <unordered_map>
#include <vector>
#include <string>
#include <functional>
#include <chrono>

#include "modbus/modbus.h"

namespace PlcCommunicate
{
    // PLC参数写入操作类型
    enum class PlcOpType : uint8_t
    {
        WRITE = 0,
        CLEAR = 1,
        RESET = 2
    };

    //  PLC参数写入模式
    enum class PlcWriteMode : uint8_t
    {
        OVERWRITE = 0,
        CHANGE = 1,
        FORCE = 2
    };

    // PLC参数写入优先级
    enum class PlcPriority : uint8_t
    {
        LOW = 0,
        MEDIUM = 1,
        HIGH = 2
    };

    struct PlcCommandEntry
    {
        uint16_t start_address;
        std::vector<uint16_t> register_values;
        PlcPriority priority;
        PlcOpType op_type;       // 0=写, 1=清零, 2=复位
        PlcWriteMode write_mode; // 0=覆盖写, 1=变化写, 2=强制写
        std::string source_id;
        std::chrono::steady_clock::time_point timestamp;
        std::chrono::milliseconds expire_duration;
    };

    class PlcScheduler
    {
    public:
        PlcScheduler(modbus_t *ctx);

    public:
        // 添加写命令到帧缓存区
        void PushCommand(const PlcCommandEntry &command);
        void Dispatch();

    private:
        //  帧缓存区：存放本帧周期内收到的所有写命令
        std::unordered_map<uint16_t, PlcCommandEntry> frame_buffer_;
        //  已写入的数据缓存区（PLC中读取到的当前实际数据，PLC寄存器区域的镜像值）
        std::unordered_map<uint16_t, std::vector<uint16_t>> plc_register_cache_;
        modbus_t *ctx_;
    };

} // namespace PlcCommunicate

#endif // PLC_SCHEDULER_HPP