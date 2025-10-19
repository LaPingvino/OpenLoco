#pragma once

#include <cstddef>
#include <fstream>
#include <string>
#include <type_traits>

namespace OpenLoco::Debug
{
    class StructureLayoutLogger
    {
    private:
        static std::ofstream logFile;
        static bool initialized;
        
    public:
        static void initialize();
        static void close();
        
        template<typename T>
        static void logStructure(const std::string& name)
        {
            if (!initialized) initialize();
            
            logFile << "=== Structure: " << name << " ===\n";
            logFile << "Size: " << sizeof(T) << " bytes (0x" << std::hex << sizeof(T) << std::dec << ")\n";
            logFile << "Alignment: " << alignof(T) << " bytes\n";
            logFile << "Is POD: " << (std::is_pod_v<T> ? "yes" : "no") << "\n";
            logFile << "Is trivially copyable: " << (std::is_trivially_copyable_v<T> ? "yes" : "no") << "\n";
            logFile << "\n";
            logFile.flush();
        }
        
        template<typename T>
        static void logMember(const std::string& structName, const std::string& memberName, size_t offset, size_t size)
        {
            if (!initialized) initialize();
            
            logFile << structName << "::" << memberName << " - Offset: " << offset << " (0x" << std::hex << offset << std::dec << "), Size: " << size << "\n";
            logFile.flush();
        }
        
        static void logSeparator(const std::string& title = "");
        static void logNote(const std::string& note);
    };
    
    // Macro to make logging easier
    #define LOG_STRUCT(T) OpenLoco::Debug::StructureLayoutLogger::logStructure<T>(#T)
    #define LOG_MEMBER(T, member) OpenLoco::Debug::StructureLayoutLogger::logMember<T>(#T, #member, offsetof(T, member), sizeof(decltype(T::member)))
    #define LOG_NOTE(note) OpenLoco::Debug::StructureLayoutLogger::logNote(note)
    #define LOG_SEPARATOR(title) OpenLoco::Debug::StructureLayoutLogger::logSeparator(title)
}