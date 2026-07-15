#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <csignal>
#include <ctime>
#include <exception>
#include <functional>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

using unitree::robot::ChannelFactory;
using unitree::robot::ChannelSubscriber;
using unitree::robot::ChannelSubscriberPtr;

namespace {

// Unitree 文档中的两个 G1 里程计 DDS 话题：
// - 高频话题约 500Hz，数据密集，持续打印时建议使用 --print-every 降采样。
// - 低频话题约 20Hz，更适合人工观察和普通诊断。
constexpr const char* kHighFrequencyTopic = "rt/odommodestate";
constexpr const char* kLowFrequencyTopic = "rt/lf/odommodestate";

// 信号处理函数只设置原子标志，真正的清理和日志输出留给主流程完成。
std::atomic_bool g_stop_requested{false};

enum class LogLevel {
    Info,
    Warn,
    Error,
};

const char* ToString(LogLevel level)
{
    switch (level) {
    case LogLevel::Info:
        return "INFO";
    case LogLevel::Warn:
        return "WARN";
    case LogLevel::Error:
        return "ERROR";
    }

    return "INFO";
}

std::string NowTimestamp()
{
    const auto now = std::chrono::system_clock::now();
    const auto time = std::chrono::system_clock::to_time_t(now);

    std::tm local_time{};
#if defined(_WIN32)
    localtime_s(&local_time, &time);
#else
    localtime_r(&time, &local_time);
#endif

    std::ostringstream out;
    out << std::put_time(&local_time, "%Y-%m-%d %H:%M:%S");
    return out.str();
}

void Log(LogLevel level, const std::string& message)
{
    static std::mutex log_mutex;
    std::lock_guard<std::mutex> lock(log_mutex);
    std::cerr << NowTimestamp() << " [" << ToString(level) << "] " << message << std::endl;
}

void SignalHandler(int)
{
    g_stop_requested.store(true);
}

struct Options {
    std::string network_interface;
    std::string topic_mode{"lf"};
    int domain_id{0};
    // samples=0 表示不按样本数退出，用于持续输出；timeout=0 表示禁用等待超时。
    int samples{5};
    int timeout_seconds{10};
    // 高频话题可达 500Hz，print_every 用来限制日志打印频率，但不影响实际接收计数。
    int print_every{1};

    bool SubscribeHigh() const
    {
        return topic_mode == "hf" || topic_mode == "both";
    }

    bool SubscribeLow() const
    {
        return topic_mode == "lf" || topic_mode == "both";
    }
};

void PrintUsage(const char* program)
{
    std::cerr
        << "Usage: " << program << " networkInterface [--topic lf|hf|both] "
        << "[--samples N] [--timeout SEC] [--domain ID] [--print-every N]\n";
}

int ParsePositiveInt(const std::string& value, const std::string& option_name)
{
    try {
        std::size_t parsed = 0;
        const int result = std::stoi(value, &parsed);
        if (parsed != value.size() || result <= 0) {
            throw std::invalid_argument("not a positive integer");
        }
        return result;
    } catch (const std::exception& error) {
        throw std::invalid_argument(option_name + " expects a positive integer: " + error.what());
    }
}

int ParseNonNegativeInt(const std::string& value, const std::string& option_name)
{
    try {
        std::size_t parsed = 0;
        const int result = std::stoi(value, &parsed);
        if (parsed != value.size() || result < 0) {
            throw std::invalid_argument("not a non-negative integer");
        }
        return result;
    } catch (const std::exception& error) {
        throw std::invalid_argument(option_name + " expects a non-negative integer: " + error.what());
    }
}

Options ParseOptions(int argc, const char** argv)
{
    if (argc < 2) {
        throw std::invalid_argument("missing networkInterface");
    }

    Options options;
    // Unitree SDK2 的 ChannelFactory 需要显式绑定网卡，例如机器人侧常见为 eth0，
    // 本机直连测试时可能是 en11。
    options.network_interface = argv[1];

    for (int i = 2; i < argc; ++i) {
        const std::string argument = argv[i];
        auto require_value = [&](const std::string& option_name) -> std::string {
            if (i + 1 >= argc) {
                throw std::invalid_argument(option_name + " requires a value");
            }
            return argv[++i];
        };

        if (argument == "--topic") {
            options.topic_mode = require_value(argument);
            if (options.topic_mode != "lf" && options.topic_mode != "hf" && options.topic_mode != "both") {
                throw std::invalid_argument("--topic must be one of: lf, hf, both");
            }
        } else if (argument == "--samples") {
            options.samples = ParseNonNegativeInt(require_value(argument), argument);
        } else if (argument == "--timeout") {
            options.timeout_seconds = ParseNonNegativeInt(require_value(argument), argument);
        } else if (argument == "--domain") {
            options.domain_id = ParseNonNegativeInt(require_value(argument), argument);
        } else if (argument == "--print-every") {
            options.print_every = ParsePositiveInt(require_value(argument), argument);
        } else if (argument == "--help" || argument == "-h") {
            PrintUsage(argv[0]);
            std::exit(0);
        } else {
            throw std::invalid_argument("unknown argument: " + argument);
        }
    }

    return options;
}

bool IsUnlimited(int value)
{
    return value == 0;
}

// Unitree 消息里的 position/velocity/rpy 都是长度为 3 的数组，这里统一格式化成一行日志。
std::string FormatTriple(const float values[3])
{
    std::ostringstream out;
    out << '[' << values[0] << ", " << values[1] << ", " << values[2] << ']';
    return out.str();
}

// Unitree 的四元数数组顺序按消息字段输出为 w/x/y/z。
std::string FormatQuaternion(const float values[4])
{
    std::ostringstream out;
    out << "[w=" << values[0] << ", x=" << values[1] << ", y=" << values[2]
        << ", z=" << values[3] << ']';
    return out.str();
}

class OdometerProbe {
public:
    explicit OdometerProbe(Options options)
        : options_(std::move(options))
    {
    }

    void Init()
    {
        std::ostringstream config;
        config << "initializing DDS channel: interface=" << options_.network_interface
               << ", domain=" << options_.domain_id
               << ", topic_mode=" << options_.topic_mode
               << ", target_samples=" << options_.samples
               << ", timeout_seconds=" << options_.timeout_seconds
               << ", print_every=" << options_.print_every;
        Log(LogLevel::Info, config.str());

        // 初始化 DDS 通道时绑定网卡和 domain。只有绑定到能看到机器人 DDS 流量的网卡，
        // 后续订阅才会收到 /odommodestate 对应的底层 DDS 数据。
        ChannelFactory::Instance()->Init(options_.domain_id, options_.network_interface);
        Log(LogLevel::Info, "DDS channel initialized");

        if (options_.SubscribeHigh()) {
            // 高频和低频订阅可以同时存在，回调统一进入 HandleMessage，
            // source 字段用于在日志中区分样本来源。
            high_subscriber_.reset(new ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(
                kHighFrequencyTopic));
            high_subscriber_->InitChannel(
                std::bind(&OdometerProbe::HandleHighFrequencyMessage, this, std::placeholders::_1), 1);
            Log(LogLevel::Info, std::string("subscribed high frequency topic: ") + kHighFrequencyTopic);
        }

        if (options_.SubscribeLow()) {
            low_subscriber_.reset(new ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(
                kLowFrequencyTopic));
            low_subscriber_->InitChannel(
                std::bind(&OdometerProbe::HandleLowFrequencyMessage, this, std::placeholders::_1), 1);
            Log(LogLevel::Info, std::string("subscribed low frequency topic: ") + kLowFrequencyTopic);
        }
    }

    bool Wait()
    {
        std::unique_lock<std::mutex> lock(mutex_);

        // 退出条件由 samples 和 timeout 两个参数组合决定：
        // samples=0 表示等待外部信号；timeout=0 表示不设置时间上限。
        if (IsUnlimited(options_.samples)) {
            if (IsUnlimited(options_.timeout_seconds)) {
                condition_.wait(lock, [&] {
                    return g_stop_requested.load();
                });
                return true;
            }

            // 持续模式但保留 timeout 时，只要在超时前收到任意样本即可认为链路可用。
            const auto deadline =
                std::chrono::steady_clock::now() + std::chrono::seconds(options_.timeout_seconds);
            return condition_.wait_until(lock, deadline, [&] {
                return g_stop_requested.load() || total_samples_ > 0;
            });
        }

        if (IsUnlimited(options_.timeout_seconds)) {
            condition_.wait(lock, [&] {
                return g_stop_requested.load() || total_samples_ >= static_cast<std::uint64_t>(options_.samples);
            });
            return true;
        }

        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(options_.timeout_seconds);
        return condition_.wait_until(lock, deadline, [&] {
            return g_stop_requested.load() || total_samples_ >= static_cast<std::uint64_t>(options_.samples);
        });
    }

    std::uint64_t TotalSamples() const
    {
        return total_samples_.load();
    }

private:
    void HandleHighFrequencyMessage(const void* message)
    {
        HandleMessage("hf", message);
    }

    void HandleLowFrequencyMessage(const void* message)
    {
        HandleMessage("lf", message);
    }

    void HandleMessage(const std::string& source, const void* message)
    {
        if (message == nullptr) {
            Log(LogLevel::Warn, "received null odometer message from " + source + " topic");
            return;
        }

        // SDK 回调传入的是 void*，这里按文档中的 SportModeState_ 类型解释。
        // 拷贝一份 state 后再打印，避免回调返回后继续引用 SDK 内部缓冲区。
        const auto state = *static_cast<const unitree_go::msg::dds_::SportModeState_*>(message);
        const auto total = ++total_samples_;

        if (ShouldPrint(total)) {
            PrintSample(source, total, state);
        }

        condition_.notify_all();
    }

    bool ShouldPrint(std::uint64_t total) const
    {
        // 已达到有限样本目标后，仍可能有回调抵达；这些样本只参与退出唤醒，不再打印。
        if (!IsUnlimited(options_.samples) && total > static_cast<std::uint64_t>(options_.samples)) {
            return false;
        }

        return ((total - 1) % static_cast<std::uint64_t>(options_.print_every)) == 0;
    }

    void PrintSample(
        const std::string& source,
        std::uint64_t total,
        const unitree_go::msg::dds_::SportModeState_& state) const
    {
        std::ostringstream out;
        // 输出字段保持物理量单位明确，方便后续接入记录器或脚本解析。
        out << "sample=" << total
            << ", source=" << source
            << ", position_m=" << FormatTriple(state.position().data())
            << ", velocity_mps=" << FormatTriple(state.velocity().data())
            << ", rpy_rad=" << FormatTriple(state.imu_state().rpy().data())
            << ", yaw_speed_radps=" << state.yaw_speed()
            << ", quaternion=" << FormatQuaternion(state.imu_state().quaternion().data());
        Log(LogLevel::Info, out.str());
    }

    Options options_;
    ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> high_subscriber_;
    ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> low_subscriber_;
    std::atomic<std::uint64_t> total_samples_{0};
    std::condition_variable condition_;
    std::mutex mutex_;
};

} // namespace

int main(int argc, const char** argv)
{
    // 支持 Ctrl+C 或外部 timeout 结束持续输出，保证主流程能记录清晰的停止原因。
    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);

    try {
        const auto options = ParseOptions(argc, argv);
        OdometerProbe probe(options);
        // 主流程很短：解析参数 -> 初始化 DDS 订阅 -> 等待样本或退出条件 -> 汇总结果。
        probe.Init();

        const bool completed = probe.Wait();
        if (g_stop_requested.load()) {
            Log(LogLevel::Warn, "stopped by signal");
            return 130;
        }

        if (!completed) {
            std::ostringstream message;
            message << "timed out waiting for odometer samples: received=" << probe.TotalSamples();
            if (!IsUnlimited(options.samples)) {
                message << ", expected=" << options.samples;
            }
            message << ", timeout_seconds=" << options.timeout_seconds;
            Log(LogLevel::Error, message.str());
            return 2;
        }

        std::ostringstream message;
        if (IsUnlimited(options.samples)) {
            message << "received odometer samples before exit: " << probe.TotalSamples();
        } else {
            message << "received requested odometer samples: " << probe.TotalSamples();
        }
        Log(LogLevel::Info, message.str());
        return 0;
    } catch (const std::exception& error) {
        Log(LogLevel::Error, std::string("odometer probe failed: ") + error.what());
        PrintUsage(argv[0]);
        return 1;
    }
}
