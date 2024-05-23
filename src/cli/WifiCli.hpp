#ifndef WIFI_CLI_HPP
#define WIFI_CLI_HPP

#include "utilities/cli/CommandInterface.hpp"
#include "drivers/serial/ISerial.hpp"

class WifiCli : public Cli::CommandInterface
{
    public:
        WifiCli(SerialComm::ISerial* pUart,
                const Cli::Command* commands,
                uint16_t numCommands,
                bool quite = false);
        ~WifiCli(){}
        bool isConnected();
        void sendWifiCommand(const char* cmd);
        void sendWifiCommand(const char* cmd, const char* value);
        void sendWifiCommand(const char* cmd, const char** values, uint8_t numVals);

        void logBackend(const char* log);
        void logBackend(const char* log, uint16_t value);

    private:
        bool initialized_ = false;
};

extern WifiCli* pWifiCli;

#endif