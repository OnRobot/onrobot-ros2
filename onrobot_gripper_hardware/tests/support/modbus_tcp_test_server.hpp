#pragma once

// ROS integration-test peer. This file is test-only, is not installed or
// bundled, and depends on POSIX/C++ rather than private Tool API headers.
// Seed values describe the supported firmware fixtures, not a physical device.
// Derived from OnRobot's internal transport test peer; maintained here so a
// checkout builds against the public binary SDK alone.

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <map>
#include <mutex>
#include <system_error>
#include <thread>
#include <vector>

class ModbusTcpTestServer
{
public:
    /// @brief Start a deterministic localhost-only Modbus server.
    ///
    /// The selectable product code lets one test fixture exercise both 2FG7
    /// and 2FG14 constructor validation without duplicating protocol logic.
    explicit ModbusTcpTestServer(uint16_t i_productCode = 0x00c0)
    {
        m_listening_socket = ::socket(AF_INET, SOCK_STREAM, 0);
        if (m_listening_socket < 0)
        {
            throwSocketError("test server socket creation failed");
        }

        sockaddr_in address{};
        address.sin_family      = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        address.sin_port        = htons(0);
        if (::bind(
                m_listening_socket,
                reinterpret_cast<sockaddr *>(&address),
                sizeof(address)) != 0)
        {
            throwSocketError("test server bind failed");
        }
        if (::listen(m_listening_socket, 1) != 0)
        {
            throwSocketError("test server listen failed");
        }

        socklen_t address_length = sizeof(address);
        if (::getsockname(
                m_listening_socket,
                reinterpret_cast<sockaddr *>(&address),
                &address_length) != 0)
        {
            throwSocketError("test server endpoint query failed");
        }
        m_port = ntohs(address.sin_port);

        m_registers[0x0600] = i_productCode;
        if (i_productCode == 0x00c1)
        {
            m_registers[0x0604] = 0x0101;
            m_registers[0x0605] = 21;
            const std::array<uint16_t, 10> hash{
                0x0e03, 0x7259, 0x7cc6, 0x6ef5, 0xf5ec,
                0x4174, 0xa143, 0x73e2, 0x3215, 0xd071
            };
            for (std::size_t index = 0; index < hash.size(); ++index)
            {
                m_registers[0x0220 + static_cast<uint16_t>(index)] = hash[index];
            }
        }
        else
        {
            m_registers[0x0604] = 0x0100;
            m_registers[0x0605] = 33;
            const std::array<uint16_t, 10> hash{
                0xb422, 0x2b25, 0x1054, 0xdec1, 0x1291,
                0x3425, 0xa623, 0x86d2, 0x8939, 0xc375
            };
            for (std::size_t index = 0; index < hash.size(); ++index)
            {
                m_registers[0x0220 + static_cast<uint16_t>(index)] = hash[index];
            }
        }
        m_registers[0x0103] = 0;
        m_registers[0x0104] = 730;
        m_registers[0x0105] = 0;
        m_registers[0x0106] = 730;
        m_registers[0x0404] = 14;
        m_registers[0x0405] = 100;
        m_registers[0x0407] = i_productCode == 0x00c1 ? 196 : 95;
        m_registers[0x0101] = 200;
        m_registers[0x0102] = 100;
        m_registers[0x0107] = static_cast<uint16_t>(static_cast<int16_t>(-40));

        m_worker = std::thread([this] {
            serve();
        });
    }

    ~ModbusTcpTestServer()
    {
        m_shutdown.store(true);
        release_request();
        const int client_socket = m_client_socket.load();
        if (client_socket >= 0)
            ::shutdown(client_socket, SHUT_RDWR);
        if (m_listening_socket >= 0)
            ::shutdown(m_listening_socket, SHUT_RDWR);
        if (m_worker.joinable())
            m_worker.join();
        if (m_listening_socket >= 0)
            ::close(m_listening_socket);
    }

    uint16_t port() const
    {
        return m_port;
    }
    void set_register(uint16_t address, uint16_t value)
    {
        std::lock_guard<std::mutex> lock(m_register_mutex);
        m_registers[address] = value;
    }

    uint16_t register_value(uint16_t address) const
    {
        std::lock_guard<std::mutex> lock(m_register_mutex);
        const auto it = m_registers.find(address);
        return it == m_registers.end() ? 0 : it->second;
    }

    /// @brief Abruptly drop the current peer while keeping the endpoint alive.
    ///
    /// The server accepts a subsequent connection on the same port, allowing
    /// deterministic identity-validation and session-recovery tests.
    void disconnect_client()
    {
        const int client_socket = m_client_socket.load();
        if (client_socket >= 0)
            ::shutdown(client_socket, SHUT_RDWR);
    }

    /// @brief Drop a paused request without sending its response.
    ///
    /// The listener remains available, so a later session recovery can
    /// establish a fresh connection on the same endpoint.
    void abort_paused_request()
    {
        {
            std::lock_guard<std::mutex> lock(m_barrier_mutex);
            m_abort_request = true;
            m_release_request = true;
            m_barrier_condition.notify_all();
        }
        disconnect_client();
    }

    void set_response_delay(std::chrono::milliseconds i_delay)
    {
        m_response_delay_ms.store(i_delay.count());
    }

    /// @brief Hold the next matching request before processing or replying.
    void pause_next_request(uint8_t i_function)
    {
        std::lock_guard<std::mutex> lock(m_barrier_mutex);
        m_pause_function = i_function;
        m_request_paused = false;
        m_release_request = false;
        m_exception_response = false;
    }

    bool wait_for_paused_request(std::chrono::milliseconds i_timeout)
    {
        std::unique_lock<std::mutex> lock(m_barrier_mutex);
        return m_barrier_condition.wait_for(lock, i_timeout, [this] {
            return m_request_paused;
        });
    }

    /// @brief Release the held request, optionally with a device exception.
    void release_request(bool i_exceptionResponse = false)
    {
        std::lock_guard<std::mutex> lock(m_barrier_mutex);
        m_exception_response = i_exceptionResponse;
        m_release_request = true;
        m_barrier_condition.notify_all();
    }

    /// @brief Process requests but omit replies to exercise response timeouts.
    void suppress_responses(bool i_suppress)
    {
        m_suppress_responses.store(i_suppress);
    }

    std::vector<std::vector<uint8_t>> requests() const
    {
        std::lock_guard<std::mutex> lock(m_function_mutex);
        return m_requests;
    }

    /// @brief Return how often a Modbus function was received by the server.
    ///
    /// Realtime tests use this to prove that command and feedback travel in
    /// one function-23 transaction rather than separate write/read requests.
    uint32_t function_count(uint8_t function) const
    {
        std::lock_guard<std::mutex> lock(m_function_mutex);
        return m_function_counts.at(function);
    }

private:
    [[noreturn]] static void throwSocketError(const char *i_operation)
    {
        throw std::system_error(errno, std::generic_category(), i_operation);
    }

    static uint16_t read_u16(const std::vector<uint8_t> &data, size_t offset)
    {
        return static_cast<uint16_t>((data[offset] << 8) | data[offset + 1]);
    }

    static void append_u16(std::vector<uint8_t> &data, uint16_t value)
    {
        data.push_back(static_cast<uint8_t>(value >> 8));
        data.push_back(static_cast<uint8_t>(value & 0xff));
    }

    static bool receive_all(int socket, uint8_t *data, size_t size)
    {
        size_t received = 0;
        while (received < size)
        {
            const auto count = ::recv(socket, data + received, size - received, 0);
            if (count <= 0)
                return false;
            received += static_cast<size_t>(count);
        }
        return true;
    }

    static bool send_all(int socket, const std::vector<uint8_t> &data)
    {
        size_t sent = 0;
        while (sent < data.size())
        {
            const auto count = ::send(socket, data.data() + sent, data.size() - sent, MSG_NOSIGNAL);
            if (count <= 0)
                return false;
            sent += static_cast<size_t>(count);
        }
        return true;
    }

    void serve_client(int i_client_socket)
    {
        while (true)
        {
            uint8_t header[7];
            if (!receive_all(i_client_socket, header, sizeof(header)))
                return;
            const uint16_t length = read_u16(std::vector<uint8_t>(header, header + sizeof(header)), 4);
            if (length < 2 || length > 254 || header[2] != 0 || header[3] != 0)
                return;
            std::vector<uint8_t> pdu(length - 1);
            if (!receive_all(i_client_socket, pdu.data(), pdu.size()))
                return;
            const auto function = pdu[0];
            // Reject incomplete requests before indexing their fields.
            if (((function == 3 || function == 6) && pdu.size() != 5) ||
                (function == 16 && (pdu.size() < 6 ||
                    pdu.size() != 6U + pdu[5] ||
                    read_u16(pdu, 3) * 2U != pdu[5])) ||
                (function == 23 && pdu.size() < 10))
                return;
            {
                std::lock_guard<std::mutex> lock(m_function_mutex);
                ++m_function_counts.at(function);
                m_requests.push_back(pdu);
            }
            bool exceptionResponse = false;
            {
                std::unique_lock<std::mutex> lock(m_barrier_mutex);
                if (m_pause_function == function)
                {
                    m_pause_function = 0;
                    m_request_paused = true;
                    m_barrier_condition.notify_all();
                    m_barrier_condition.wait(lock, [this] {
                        return m_release_request;
                    });
                    exceptionResponse = m_exception_response;
                    if (m_abort_request)
                    {
                        m_abort_request = false;
                        return;
                    }
                }
            }
            std::vector<uint8_t> response_pdu;

            if (exceptionResponse)
            {
                response_pdu = { static_cast<uint8_t>(function | 0x80U), 4 };
            }
            else if (function == 3)
            {
                const auto address = read_u16(pdu, 1);
                const auto count   = read_u16(pdu, 3);
                response_pdu.push_back(3);
                response_pdu.push_back(static_cast<uint8_t>(count * 2));
                {
                    std::lock_guard<std::mutex> lock(m_register_mutex);
                    for (uint16_t index = 0; index < count; ++index)
                    {
                        append_u16(response_pdu, m_registers[address + index]);
                    }
                }
            }
            else if (function == 6)
            {
                const auto address = read_u16(pdu, 1);
                {
                    std::lock_guard<std::mutex> lock(m_register_mutex);
                    m_registers[address] = read_u16(pdu, 3);
                }
                response_pdu = pdu;
            }
            else if (function == 16)
            {
                const auto address = read_u16(pdu, 1);
                const auto count   = read_u16(pdu, 3);
                {
                    std::lock_guard<std::mutex> lock(m_register_mutex);
                    for (uint16_t index = 0; index < count; ++index)
                    {
                        m_registers[address + index] = read_u16(pdu, 6 + index * 2);
                    }
                }
                response_pdu.push_back(16);
                append_u16(response_pdu, address);
                append_u16(response_pdu, count);
            }
            else if (function == 23)
            {
                const auto read_address  = read_u16(pdu, 1);
                const auto read_count    = read_u16(pdu, 3);
                const auto write_address = read_u16(pdu, 5);
                const auto write_count   = read_u16(pdu, 7);
                const auto byte_count    = pdu.at(9);
                if (byte_count != write_count * 2 || pdu.size() != 10U + byte_count)
                {
                    return;
                }

                response_pdu.push_back(23);
                response_pdu.push_back(static_cast<uint8_t>(read_count * 2));
                {
                    std::lock_guard<std::mutex> lock(m_register_mutex);
                    for (uint16_t index = 0; index < write_count; ++index)
                    {
                        m_registers[write_address + index] = read_u16(pdu, 10 + index * 2);
                    }
                    for (uint16_t index = 0; index < read_count; ++index)
                    {
                        append_u16(response_pdu, m_registers[read_address + index]);
                    }
                }
            }
            else
            {
                return;
            }

            std::vector<uint8_t> response(header, header + 4);
            response.push_back(0);
            response.push_back(static_cast<uint8_t>(response_pdu.size() + 1));
            response.push_back(header[6]);
            response.insert(response.end(), response_pdu.begin(), response_pdu.end());
            if (m_suppress_responses.load())
            {
                continue;
            }
            const auto response_delay_ms = m_response_delay_ms.load();
            if (response_delay_ms > 0)
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(response_delay_ms));
            if (!send_all(i_client_socket, response))
                return;
        }
    }

    void serve()
    {
        while (!m_shutdown.load())
        {
            const int client_socket =
                ::accept(m_listening_socket, nullptr, nullptr);
            if (client_socket < 0)
                return;
            m_client_socket.store(client_socket);
            serve_client(client_socket);
            m_client_socket.store(-1);
            ::close(client_socket);
        }
    }

    int m_listening_socket{ -1 };
    std::atomic<int> m_client_socket{ -1 };
    std::atomic_bool m_shutdown{ false };
    std::atomic<std::int64_t> m_response_delay_ms{ 0 };
    std::atomic_bool m_suppress_responses{ false };
    std::mutex m_barrier_mutex;
    std::condition_variable m_barrier_condition;
    uint8_t m_pause_function{ 0 };
    bool m_request_paused{ false };
    bool m_release_request{ false };
    bool m_exception_response{ false };
    bool m_abort_request{ false };
    uint16_t m_port{ 0 };
    std::map<uint16_t, uint16_t> m_registers;
    mutable std::mutex m_register_mutex;
    std::array<uint32_t, 256> m_function_counts{};
    std::vector<std::vector<uint8_t>> m_requests;
    mutable std::mutex m_function_mutex;
    std::thread m_worker;
};

