#pragma once
// LiveVideo.hpp - живое превью на землю: уменьшенный кадр + рамки детектора.
//
// ОТДЕЛЬНО ОТ НАХОДОК. Находки идут через ReliableLink: очередь на карте,
// ACK, переотправка. Видео так слать нельзя и не нужно: устаревший кадр
// бесполезен, а очередь из кадров забила бы канал и задержала находки.
// Поэтому здесь своё соединение (порт 5701), без подтверждений, и в нём
// живёт только САМЫЙ СВЕЖИЙ кадр: не успели отправить - его заменил новый.
//
// ПОТОК ИНФЕРЕНСА НЕ ЖДЁТ. push() раз в 1/fps уменьшает кадр (~2 мс) и
// кладёт его в ячейку; JPEG и send() - в своём потоке. Чаще fps push()
// возвращается сразу.
//
// РАМКИ НЕ РИСУЮТСЯ НА КАДРЕ: они идут числами в заголовке, браузер рисует
// их сам - так их можно скрывать по классам и подписывать по-русски.
//
// ФОРМАТ (little-endian):
//   0..3  "SARV"
//   4..7  длина заголовка JSON
//   8..11 длина JPEG
//   далее заголовок JSON, затем JPEG
// Заголовок: {"t_us":..,"w":640,"h":360,"dets":[[класс,увер,x,y,w,h],...]}
// x, y, w, h - доли кадра (0..1), левый верхний угол и размер.
//
// Настройки без пересборки: SAR_VIDEO=0 выключить, SAR_VIDEO_FPS=5,
// SAR_VIDEO_WIDTH=640, SAR_VIDEO_JPEG=60.

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <opencv2/opencv.hpp>

#include "RknnYolo.hpp"

namespace sar {

class LiveVideo {
public:
    LiveVideo(std::string ip, uint16_t port, double fps = 5.0, int width = 640, int quality = 60)
        : ip_(std::move(ip)), port_(port), fps_(fps), width_(width), quality_(quality) {}

    ~LiveVideo() { stop(); }

    void start() {
        if (running_) return;
        running_ = true;
        th_ = std::thread([this] { loop(); });
    }

    void stop() {
        if (!running_) return;
        {
            std::lock_guard<std::mutex> lk(mu_);
            running_ = false;
        }
        cv_.notify_all();
        if (th_.joinable()) th_.join();
        close_sock();
    }

    bool connected() const { return connected_; }
    uint64_t sent() const { return sent_; }

    // Из потока инференса. Не блокируется на сети.
    void push(const cv::Mat& bgr, const std::vector<Detection>& dets, uint64_t t_us) {
        if (!running_ || bgr.empty()) return;
        const auto now = std::chrono::steady_clock::now();
        if (now - last_push_ < std::chrono::duration<double>(1.0 / fps_)) return;
        last_push_ = now;

        cv::Mat small;
        if (bgr.cols > width_) {
            const int h = int(std::lround(double(bgr.rows) * width_ / bgr.cols));
            cv::resize(bgr, small, cv::Size(width_, h), 0, 0, cv::INTER_AREA);
        } else {
            small = bgr.clone();
        }

        std::ostringstream js;
        js.precision(4);
        js << "{\"t_us\":" << t_us << ",\"w\":" << small.cols << ",\"h\":" << small.rows
           << ",\"dets\":[";
        const double W = bgr.cols, H = bgr.rows;
        for (size_t i = 0; i < dets.size(); ++i) {
            const auto& d = dets[i];
            if (i) js << ",";
            js << "[" << d.class_id << "," << d.confidence << ","
               << (d.cx - d.w / 2) / W << "," << (d.cy - d.h / 2) / H << ","
               << d.w / W << "," << d.h / H << "]";
        }
        js << "]}";

        {
            std::lock_guard<std::mutex> lk(mu_);
            img_ = std::move(small);
            hdr_ = js.str();
            fresh_ = true;
        }
        cv_.notify_one();
    }

private:
    void loop() {
        auto next_try = std::chrono::steady_clock::now();
        while (true) {
            cv::Mat img;
            std::string hdr;
            {
                std::unique_lock<std::mutex> lk(mu_);
                cv_.wait_for(lk, std::chrono::seconds(1), [this] { return fresh_ || !running_; });
                if (!running_) return;
                if (!fresh_) continue;
                img = std::move(img_);
                hdr = std::move(hdr_);
                fresh_ = false;
            }

            // Земли нет - кадр просто выбрасываем, переподключаемся не чаще раза в 3 с.
            if (!connected_) {
                if (std::chrono::steady_clock::now() < next_try) continue;
                next_try = std::chrono::steady_clock::now() + std::chrono::seconds(3);
                if (!connect_once()) continue;
                std::cout << "видео: земля " << ip_ << ":" << port_ << " на связи\n" << std::flush;
            }

            std::vector<uchar> jpg;
            cv::imencode(".jpg", img, jpg, {cv::IMWRITE_JPEG_QUALITY, quality_});

            uint8_t head[12];
            std::memcpy(head, "SARV", 4);
            const uint32_t hl = uint32_t(hdr.size()), jl = uint32_t(jpg.size());
            std::memcpy(head + 4, &hl, 4);       // little-endian: плата и ноутбук такие же
            std::memcpy(head + 8, &jl, 4);
            if (!send_all(head, sizeof(head)) ||
                !send_all(reinterpret_cast<const uint8_t*>(hdr.data()), hdr.size()) ||
                !send_all(jpg.data(), jpg.size())) {
                std::cerr << "видео: связь с землёй потеряна, переподключусь\n";
                close_sock();
                continue;
            }
            ++sent_;
        }
    }

    bool connect_once() {
        close_sock();
        sock_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (sock_ < 0) return false;
        sockaddr_in a{};
        a.sin_family = AF_INET;
        a.sin_port   = htons(port_);
        if (::inet_pton(AF_INET, ip_.c_str(), &a.sin_addr) != 1) { close_sock(); return false; }
        // Таймаут отправки: на забитом канале лучше порвать и переподключиться,
        // чем показывать кадры с задержкой в десятки секунд.
        timeval tv{2, 0};
        ::setsockopt(sock_, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
        int one = 1;
        ::setsockopt(sock_, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
        // Маленький буфер отправки: в ядре не копятся старые кадры.
        int sndbuf = 64 * 1024;
        ::setsockopt(sock_, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));
        if (::connect(sock_, reinterpret_cast<sockaddr*>(&a), sizeof(a)) != 0) {
            close_sock();
            return false;
        }
        connected_ = true;
        return true;
    }

    void close_sock() {
        if (sock_ >= 0) { ::close(sock_); sock_ = -1; }
        connected_ = false;
    }

    bool send_all(const uint8_t* p, size_t n) {
        while (n) {
            const ssize_t k = ::send(sock_, p, n, MSG_NOSIGNAL);
            if (k <= 0) return false;
            p += k; n -= size_t(k);
        }
        return true;
    }

    std::string ip_;
    uint16_t    port_;
    double      fps_;
    int         width_, quality_;

    std::thread th_;
    std::mutex  mu_;
    std::condition_variable cv_;
    std::atomic<bool> running_{false};
    std::atomic<bool> connected_{false};
    std::atomic<uint64_t> sent_{0};
    int sock_ = -1;

    // Ячейка "последний кадр": пишет поток инференса, читает поток отправки.
    cv::Mat     img_;
    std::string hdr_;
    bool        fresh_ = false;
    std::chrono::steady_clock::time_point last_push_{};
};

} // namespace sar
