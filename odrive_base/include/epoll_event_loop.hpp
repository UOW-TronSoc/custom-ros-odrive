#ifndef EPOLL_EVENT_LOOP_HPP
#define EPOLL_EVENT_LOOP_HPP

/*
 * Minimal epoll loop used by the CAN thread (main.cpp).
 *
 * EpollEventLoop
 *   Registers fds (SocketCAN socket, stop eventfd, EpollEvent eventfds) and
 *   dispatches callbacks from run_until_empty() until request_stop().
 *
 * EpollEvent
 *   Cross-thread signal: ROS thread calls set() → writes eventfd → CAN thread
 *   runs the registered callback (e.g. ctrl_msg_callback). This keeps SocketCAN
 *   TX on one thread while ROS callbacks stay non-blocking.
 */

#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <atomic>
#include <iostream>
#include <functional>
#include <vector>
#include <unistd.h>

using std::placeholders::_1;
using Callback = std::function<void(uint32_t)>;

class EpollEventLoop {
public:
    struct EventContext {
        int fd;
        Callback callback;
    };

    using EvtId = EventContext*;

    EpollEventLoop();

    ~EpollEventLoop();

    // If p_evt is non-null, stores the context pointer for later deregister.
    bool register_event(EvtId* p_evt, int fd, uint32_t events, const Callback& callback);

    bool deregister_event(EvtId evt);

    // Blocks in epoll_wait until request_stop() or no events remain.
    bool run_until_empty();

    // Wakes run_until_empty() and makes it return so the loop thread can be joined.
    void request_stop();

    // Null out a context in the current triggered batch (safe during callback).
    void drop_event(EvtId evt);

private:
    static constexpr size_t kMaxEventsPerIteration = 16;
    int epollfd = -1;
    int stop_fd_ = -1;  // eventfd written by request_stop()
    std::atomic<bool> stop_requested_{false};
    size_t n_events_ = 0;
    int n_triggered_events_ = 0;
    struct epoll_event triggered_events_[kMaxEventsPerIteration];
};

class EpollEvent {
public:
    bool init(EpollEventLoop* event_loop, const Callback& callback);
    void deinit();

    // Signal from any thread: increments eventfd counter → on_trigger → callback_.
    bool set();

private:
    void on_trigger(uint32_t event_id);

    EpollEventLoop* event_loop_;
    int fd_ = -1;
    EpollEventLoop::EventContext* evt_;
    Callback callback_;
};


#endif // EPOLL_EVENT_LOOP_HPP
