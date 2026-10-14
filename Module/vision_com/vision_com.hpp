/**
 * @file vision_com.hpp
 * @brief 视觉/上位机通信发送封装（上行帧）
 */
#pragma once

#include "UartPort.hpp"
#include "vision_protocol.hpp"

class VisionCom
{
public:
    bool init(UartPort *port);

    /* 通用无payload上行帧 (0x0201~0x0206, 0x0301) */
    bool sendEvent(VisionProtocol::UpCode code);

    /* 请求下一步路径动作: AA 55 00 0A 03 01 A3 B9 55 AA */
    bool sendPathRequestNext() { return sendEvent(VisionProtocol::UP_REQ_NEXT); }

    /* 请求下一步(带路径索引), index 按小端填入payload */
    bool sendPathRequestNextIndexed(uint16_t index);

    bool sendActionDone() { return sendEvent(VisionProtocol::UP_ACTION_DONE); }
    bool sendEnterHigh() { return sendEvent(VisionProtocol::UP_ENTER_HIGH); }
    bool sendDescendDone() { return sendEvent(VisionProtocol::UP_DESCEND_DONE); }
    bool sendStepUpDone() { return sendEvent(VisionProtocol::UP_STEP_UP_DONE); }
    bool sendStepDownDone() { return sendEvent(VisionProtocol::UP_STEP_DOWN_DONE); }
    bool sendEstopDone() { return sendEvent(VisionProtocol::UP_ESTOP_DONE); }

private:
    UartPort *port_{nullptr};
    // writeDma 内部会把数据拷入自己的DMA双缓冲, 这里无需DMA段属性
    uint8_t tx_buf_[VisionProtocol::kMaxFrameLen]{};
};