#include <unordered_map>
#include <vector>
#include <algorithm>
#include <iostream>
#include <chrono>

#include "plc_scheduler.hpp"

namespace PlcCommunicate
{
    PlcScheduler::PlcScheduler(modbus_t *ctx) : ctx_(ctx)
    {
    }

    void
    PlcScheduler::PushCommand(const PlcCommandEntry &command)
    {
        // 检查命令是否过期
        auto now = std::chrono::steady_clock::now();
        if (now - command.timestamp > command.expire_duration)
        {
            std::cout << "Command from source " << command.source_id << " has expired and will be discarded." << std::endl;
            return;
        }

        auto it = frame_buffer_.find(command.start_address);
        if (it != frame_buffer_.end())
        {
            // 如果存在相同起始地址的命令，比较优先级
            if (command.priority > it->second.priority)
            {
                frame_buffer_[command.start_address] = command;
            }
            else if (command.priority == it->second.priority)
            {
                // 如果优先级相同，选择最新的命令
                if (command.timestamp > it->second.timestamp)
                {
                    frame_buffer_[command.start_address] = command;
                }
            }
            // 否则，忽略新命令
        }
        else
        {
            // 新命令，直接添加
            frame_buffer_[command.start_address] = command;
        }
    }

    void
    PlcScheduler::Dispatch()
    {
        auto now = std::chrono::steady_clock::now();
        if (frame_buffer_.empty())
            return;

        // Step 1: 按地址排序，方便后续合并
        std::vector<PlcCommandEntry> sorted_cmds;
        for (const auto &entry : frame_buffer_)
        {
            sorted_cmds.push_back(entry.second);
        }
        std::sort(sorted_cmds.begin(), sorted_cmds.end(),
                  [](const PlcCommandEntry &a, const PlcCommandEntry &b)
                  {
                      return a.start_address < b.start_address;
                  });

        // Step 2: 遍历处理
        size_t i = 0;
        while (i < sorted_cmds.size())
        {
            auto command = sorted_cmds[i];

            // 过期检查
            if (command.expire_duration.count() > 0 &&
                now - command.timestamp > command.expire_duration)
            {
                std::cout << "[Scheduler] Command from " << command.source_id
                          << " expired, clearing." << std::endl;
                std::fill(command.register_values.begin(),
                          command.register_values.end(), 0);
            }

            // 操作类型处理
            switch (command.op_type)
            {
            case PlcOpType::WRITE:
                // 正常写，保持原值
                break;
            case PlcOpType::CLEAR:
            case PlcOpType::RESET:
                std::fill(command.register_values.begin(),
                          command.register_values.end(), 0);
                break;
            }

            // 写入模式判断
            bool should_write = false;
            switch (command.write_mode)
            {
            case PlcWriteMode::OVERWRITE:
                should_write = true;
                break;
            case PlcWriteMode::CHANGE:
            {
                auto it = plc_register_cache_.find(command.start_address);
                if (it == plc_register_cache_.end() ||
                    it->second != command.register_values)
                {
                    should_write = true;
                }
                break;
            }
            case PlcWriteMode::FORCE:
                should_write = true;
                break;
            }

            if (!should_write)
            {
                ++i;
                continue; // 跳过该命令
            }

            // 连续命令合并
            uint16_t base_addr = command.start_address;
            std::vector<uint16_t> merged_values(command.register_values);

            size_t j = i + 1;
            while (j < sorted_cmds.size())
            {
                const auto &next_cmd = sorted_cmds[j];
                uint16_t expected_next = base_addr + merged_values.size();

                // 只有在连续 & 模式允许的情况下才合并
                if (next_cmd.start_address == expected_next)
                {
                    merged_values.insert(merged_values.end(),
                                         next_cmd.register_values.begin(),
                                         next_cmd.register_values.end());
                    ++j;
                }
                else
                {
                    break; // 不连续，中断合并
                }
            }

            // 调用外部写接口
            bool success = false;
            int num = 0;
            if (merged_values.size() > 1)
            {
                num = modbus_write_registers(ctx_,
                                             base_addr,
                                             merged_values.size(),
                                             merged_values.data());
                success = num != -1;
            }
            else
            {
                num = modbus_write_register(ctx_,
                                            base_addr,
                                            merged_values[0]);
                success = num != -1;
            }

            if (success)
            {
                // 更新缓存
                plc_register_cache_[base_addr] = merged_values;
                // std::cout << "[Scheduler] Wrote " << merged_values.size()
                //           << " registers at " << base_addr << std::endl;
            }
            else
            {
                std::cerr << "[Scheduler] Write failed at addr " << base_addr
                          << std::endl;
            }

            i = j; // 处理到下一个段
        }

        // Step 3: 清理帧缓冲
        frame_buffer_.clear();
    }

} // namespace PlcCommunicate
