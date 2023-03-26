#include "FileSystemClient.h"
#include <filesystem>
#include <sstream>
#include <fstream>
#include <unistd.h>
#include <stdbool.h>
#include <iostream>
namespace mandeye {

    FileSystemClient::FileSystemClient(const std::string& repository):
        m_repository(repository){
        m_nextId = GetIdFromManifest();
    }
    nlohmann::json FileSystemClient::produceStatus()
    {
        nlohmann::json data;
        data["FileSystemClient"]["repository"] = m_repository;
        float free_mb = CheckAvailableSpace();
        data["FileSystemClient"]["free_megabytes"] = free_mb;
        data["FileSystemClient"]["free_str"] = ConvertToText(free_mb);
        data["FileSystemClient"]["error"] = m_error;
        data["FileSystemClient"]["m_nextId"] = m_nextId;
        data["FileSystemClient"]["writable"] = GetIsWritable();
        data["FileSystemClient"]["dirs"] = GetDirectories();
        return data;
    }

    //! Test is writable
    float FileSystemClient::CheckAvailableSpace()
    {
        std::error_code ec;
        const std::filesystem::space_info si = std::filesystem::space(m_repository, ec);
        if (ec.value()== 0) {
            const float f= static_cast<float>(si.free) / (1024 * 1024 );
            return std::round(f);
        }
        return -1.f;
    }


    std::string FileSystemClient::ConvertToText(float mb)
    {
        std::stringstream tmp;
        tmp << std::setprecision(1) <<std::fixed << mb/1024 << "GB";
        return tmp.str();
    }

    int32_t FileSystemClient::GetIdFromManifest()
    {
        std::filesystem::path manifest = std::filesystem::path(m_repository)/std::filesystem::path(manifestFilename);
        std::unique_lock<std::mutex> lck(m_mutex);

        std::ifstream manifestFstream;
        manifestFstream.open(manifest.c_str());
        if (manifestFstream.good() && manifestFstream.is_open())
        {
            uint32_t id {0};
            manifestFstream >> id;
            return id++;
        }
        std::ofstream manifestOFstream;
        manifestOFstream.open(manifest.c_str());
        if (manifestOFstream.good() && manifestOFstream.is_open())
        {
            uint32_t id {0};
            manifestOFstream << id << std::endl;
            return id++;
        }
        return -1;
    }

    int32_t FileSystemClient::GetNextIdFromManifest()
    {
        std::filesystem::path manifest = std::filesystem::path(m_repository)/std::filesystem::path(manifestFilename);
        int32_t id = GetIdFromManifest();
        id++;
        m_nextId = id;
        std::ofstream manifestOFstream;
        manifestOFstream.open(manifest.c_str());
        if (manifestOFstream.good() && manifestOFstream.is_open())
        {
            manifestOFstream << id << std::endl;
            return id;
        }
        return id;
    }

    std::string FileSystemClient::CreateDirectoryForExperiment(){
        std::string ret;

        if (GetIsWritable()){
            auto id = GetNextIdFromManifest();
            char dirName [256];
            snprintf(dirName,256, "dataset_%04d", id);
            std::filesystem::path newDirPath = std::filesystem::path(m_repository)/std::filesystem::path(dirName);
            std::cout << "Creating directory " << newDirPath.string() << std::endl;
            std::error_code ec;
            std::filesystem::create_directories(newDirPath, ec);
            m_error = ec.message();
            if (ec.value()==0){
                return newDirPath.string();
            }
        }
        return "";
    }

    std::vector<std::string> FileSystemClient::GetDirectories()
    {
        std::unique_lock<std::mutex> lck(m_mutex);
        std::vector<std::string> fn;
        for (const auto & entry : std::filesystem::directory_iterator(m_repository)) {
            fn.push_back(entry.path());
        }
        std::sort(fn.begin(), fn.end());
        return fn;
    }

    bool FileSystemClient::GetIsWritable(){
        if(access(m_repository.c_str(), W_OK) == 0) {
            return true;
        } else {
            return false;
        }
    }

} // namespace mandeye