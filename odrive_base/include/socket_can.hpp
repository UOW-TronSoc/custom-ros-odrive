#ifndef SOCKET_CAN_HPP
#define SOCKET_CAN_HPP

/*
 * Thin non-blocking SocketCAN wrapper registered with EpollEventLoop.
 *
 * On EPOLLIN, frames are read until EAGAIN and passed to FrameProcessor
 * (CustomODriveNode::recv_callback). TX is a plain write() of can_frame.
 *
 * The SocketCAN interface itself (bitrate, up/down) is owned by the host OS /
 * rover scripts — this class only opens/binds an already-configured iface.
 */

#include "epoll_event_loop.hpp"
#include <linux/can.h>
#include <linux/can/raw.h>
#include <string>
#include <functional>

using FrameProcessor = std::function<void(const can_frame&)>;

class SocketCanIntf {
public:
    // Open PF_CAN/SOCK_RAW non-blocking, bind to `interface`, register with epoll.
    bool init(const std::string& interface, EpollEventLoop* event_loop, FrameProcessor frame_processor);
    void deinit();

    // Returns false on write failure (logs to stderr).
    bool send_can_frame(const can_frame& frame);

    // Drain ready frames; called from the epoll callback.
    bool read_nonblocking();

private:
    std::string interface_;
    int socket_id_ = -1;
    EpollEventLoop* event_loop_ = nullptr;
    EpollEventLoop::EvtId socket_evt_id_;
    FrameProcessor frame_processor_;
    bool broken_ = false;

    void on_socket_event(uint32_t mask);
    void process_can_frame(const can_frame& frame) {
        frame_processor_(frame);
    }
};

#endif  // SOCKET_CAN_HPP
