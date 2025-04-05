#ifndef __PARAMETER_PERSISTENCE_HPP__
#define __PARAMETER_PERSISTENCE_HPP__

#include <string>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <sstream>
#include "xpack/json.h"

#define PARAM_INIT(...) XPACK(M(__VA_ARGS__))
namespace PPcem
{
    template <typename ParamStructType>
    class ParameterPersistence
    {
    public:
        ParameterPersistence(std::string file_dir) : sourceFile_(file_dir)
        {
            writeThread_ = std::thread(&ParameterPersistence::WriteFile, this);
        }

        bool Init()
        {
            printf("[ParameterPersistence]: load source file: %s\n", sourceFile_.c_str());
            backupFile_ = sourceFile_ + "_backup";
            if (this->LoadFile(sourceFile_, paramStruct_))
            {
                printf("[ParameterPersistence]: load source file success!\n");
                this->CopyFile(sourceFile_.c_str(), backupFile_.c_str());
            }
            else
            {
                printf("[ParameterPersistence]: load source file error, try load backup file.\n");
                if (this->LoadFile(backupFile_, paramStruct_))
                {
                    printf("[ParameterPersistence]: load backup file success, replace source file with backup file.\n");
                    this->CopyFile(backupFile_.c_str(), sourceFile_.c_str());
                }
                else
                {
                    printf("[ParameterPersistence]: load backup file error, will be used defaule value!\n");
                    paramStruct_ = ParamStructType();
                    return false;
                }
            }
            paramJsonStr_ = xpack::json::encode(paramStruct_, 0, 2, ' ');
            return true;
        }

        ParamStructType &GetParam()
        {
            return paramStruct_;
        }

        ~ParameterPersistence()
        {
            gExit_ = true;
            cv_.notify_all();
            writeThread_.join();
        }

        void Update()
        {
            std::unique_lock<std::mutex> lock(mtxParamJson_);
            paramJsonStr_ = xpack::json::encode(paramStruct_, 0, 2, ' ');
            cv_.notify_all();
            lock.unlock();
        }

    private:
        std::string sourceFile_;
        std::string backupFile_;
        ParamStructType paramStruct_;
        std::mutex mtxParamJson_;
        std::thread writeThread_;
        std::condition_variable cv_;
        std::string paramJsonStr_;
        std::atomic<bool> gExit_ = false;

        void CopyFile(const char *src, const char *des)
        {
            std::ifstream s_src(src, std::ios::binary);
            std::ofstream sDes(des, std::ios::binary);
            sDes << s_src.rdbuf() << std::endl;
        }
        std::vector<std::string> Split(const std::string &s, char delimiter)
        {
            std::vector<std::string> tokens;
            std::string token;
            std::istringstream tokenStream(s);
            while (std::getline(tokenStream, token, delimiter))
            {
                tokens.push_back(token);
            }
            return tokens;
        }
        std::string GetFileType(std::string path)
        {
            std::vector<std::string> vec = this->Split(path, '.');
            return vec.at(vec.size() - 1);
        }

        bool LoadFile(std::string file, ParamStructType &parameter)
        {
            if (this->GetFileType(file) == "json" || this->GetFileType(file) == "json_backup")
            {
                std::ifstream tStream(file);
                if (!tStream.is_open())
                {
                    printf("[ParameterPersistence]: LoadFile error\n");
                    return false;
                }

                std::string fileContentStr((std::istreambuf_iterator<char>(tStream)), std::istreambuf_iterator<char>());
                try
                {
                    xpack::json::decode(fileContentStr, parameter);
                }
                catch (const std::exception &e)
                {
                    std::cerr << "[ParameterPersistence]: json_to_struct error, " << e.what() << '\n';
                    return false;
                }
                return true;
            }
            else
            {
                printf("[ParameterPersistence]: file type error, only support .json/.json_backup file\n");
            }
            return false;
        }
        void WriteFile()
        {
            while (!gExit_)
            {
                std::unique_lock<std::mutex> lock(mtxParamJson_);
                cv_.wait(lock);
                int fd = open(sourceFile_.c_str(), O_WRONLY | O_CREAT, 00700);
                write(fd, paramJsonStr_.c_str(), paramJsonStr_.size());
                write(fd, "\n", 1);
                fsync(fd);
                close(fd);
                printf("[ParameterPersistence]: write file success!\n");
            }
        }
    };
}

#endif