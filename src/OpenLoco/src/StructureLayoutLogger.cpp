#include "StructureLayoutLogger.h"
#include <iostream>
#include <iomanip>

namespace OpenLoco::Debug
{
    std::ofstream StructureLayoutLogger::logFile;
    bool StructureLayoutLogger::initialized = false;
    
    void StructureLayoutLogger::initialize()
    {
        if (initialized) return;
        
        logFile.open("structure_layout_64bit.log", std::ios::out | std::ios::trunc);
        if (logFile.is_open())
        {
            logFile << "OpenLoco 64-bit Structure Layout Analysis\n";
            logFile << "=========================================\n";
            logFile << "Generated to help fix memory layout issues for 64-bit builds\n";
            logFile << "Compare these values with the static_assert statements in the code\n\n";
            initialized = true;
        }
        else
        {
            std::cerr << "Failed to open structure layout log file\n";
        }
    }
    
    void StructureLayoutLogger::close()
    {
        if (logFile.is_open())
        {
            logFile << "\n=== End of Structure Layout Analysis ===\n";
            logFile.close();
        }
        initialized = false;
    }
    
    void StructureLayoutLogger::logSeparator(const std::string& title)
    {
        if (!initialized) initialize();
        
        logFile << "\n";
        if (!title.empty())
        {
            logFile << "--- " << title << " ---\n";
        }
        else
        {
            logFile << "----------------------------------------\n";
        }
        logFile.flush();
    }
    
    void StructureLayoutLogger::logNote(const std::string& note)
    {
        if (!initialized) initialize();
        
        logFile << "NOTE: " << note << "\n";
        logFile.flush();
    }
}