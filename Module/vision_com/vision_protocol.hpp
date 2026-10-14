/**
 * @file vision_protocol.hpp
 * @brief 视觉/上位机串口协议解析器（纯协议层，不依赖硬件）
 *
 * 帧结构: AA 55 | LEN(2B,BE,总帧长) | CODE(2B,BE) | CRC16(2B,BE) | PAYLOAD | 55 AA
 * CRC16-MODBUS(反向多项式0xA001, 初值0xFFFF),
 * 范围 = 除CRC字段自身外的全部字节(含帧头帧尾), 结果大端填入。
 * payload 内的 int16/uint16 为小端。
 */

#pragma once
#include <cstddef>
#include <cstdint>

class VisionProtocol
{
public:
    static constexpr uint8_t kHeader1 = 0xAA;
    static constexpr uint8_t kHeader2 = 0x55;
    static constexpr uint8_t kTail1 = 0x55;
    static constexpr uint8_t kTail2 = 0xAA;
    static constexpr uint16_t kFrameOverhead = 10; // 头2+长2+码2+CRC2+尾2
    static constexpr uint16_t kMaxPayload = 56;
    static constexpr uint16_t kMaxFrameLen = kFrameOverhead + kMaxPayload;

    /* 下行业务码 (上位机 -> STM32) */
    enum DownCode : uint16_t
    {
        DOWN_NAV_POSE = 0x0101, // 位姿: x,y,yaw 各int16 LE
        DOWN_ESTOP = 0x0105,    // 紧急停止
        DOWN_PATH_FORWARD = 0x0311,
        DOWN_PATH_BACKWARD = 0x0312,
        DOWN_PATH_TURN_LEFT_90 = 0x0313,
        DOWN_PATH_TURN_RIGHT_90 = 0x0314,
        DOWN_PATH_SHIFT_LEFT = 0x0315,
        DOWN_PATH_SHIFT_RIGHT = 0x0316,
        DOWN_PATH_GRAB_LOW_KFS = 0x0317,
        DOWN_PATH_GRAB_MID_KFS = 0x0318,
        DOWN_PATH_GRAB_HIGH_KFS = 0x0319,
        DOWN_PATH_REPLACE_KFS = 0x031A,
        DOWN_PATH_NO_COMMAND = 0x031B,
        DOWN_PATH_TURN_AROUND_180 = 0x031C,
        DOWN_PATH_MOVE_TO_COL1 = 0x031E,
        DOWN_PATH_MOVE_TO_COL2 = 0x031F,
        DOWN_PATH_MOVE_TO_COL3 = 0x0320,
    };

    /* 上行业务码 (STM32 -> 上位机) */
    enum UpCode : uint16_t
    {
        UP_ACTION_DONE = 0x0201,      // 动作完成
        UP_ENTER_HIGH = 0x0202,       // 进入高位
        UP_DESCEND_DONE = 0x0203,     // 下降完成
        UP_STEP_UP_DONE = 0x0204,     // 上台阶完成
        UP_STEP_DOWN_DONE = 0x0205,   // 下台阶完成
        UP_ESTOP_DONE = 0x0206,       // 急停完成
        UP_REQ_NEXT = 0x0301,         // 请求下一步路径动作(无payload)
        UP_REQ_NEXT_INDEXED = 0x031D, // 请求下一步(payload: uint16 LE 路径索引)
    };

    struct Frame
    {
        uint16_t code{0};
        uint16_t payload_len{0};
        uint8_t payload[kMaxPayload]{};
    };

    void reset()
    {
        state_ = State::HEAD1;
        rx_len_ = 0;
        frame_len_ = 0;
        crc_rx_ = 0;
    }

    /**
     * @brief 逐字节喂入解析
     * @param byte 接收到的字节
     * @param out  帧完整且CRC正确时填充
     * @return true = 成功解析出一帧
     */
    bool processByte(uint8_t byte, Frame &out);

    /**
     * @brief CRC16-MODBUS, 可分段续算(传入上一次的crc)
     */
    static uint16_t crc16Modbus(const uint8_t *data, size_t len,
                                uint16_t crc = 0xFFFF);

    /**
     * @brief 打包一帧
     * @return 帧总长度; payload超长返回0
     */
    static uint16_t packFrame(uint16_t code, const uint8_t *payload,
                              uint16_t payload_len, uint8_t *out_buf);

private:
    enum class State : uint8_t
    {
        HEAD1,
        HEAD2,
        LEN_H,
        LEN_L,
        CODE_H,
        CODE_L,
        CRC_H,
        CRC_L,
        PAYLOAD,
        TAIL1,
        TAIL2
    };

    State state_{State::HEAD1};
    uint16_t rx_len_{0};    // rx_buf_ 中已收字节数
    uint16_t frame_len_{0}; // LEN 字段给出的总帧长
    uint16_t crc_rx_{0};    // 帧内携带的 CRC
    uint8_t rx_buf_[kMaxFrameLen]{};
};
