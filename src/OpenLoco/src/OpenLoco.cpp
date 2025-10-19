#include "CommandLine.h"
#include "Scenario.h"
#include <algorithm>
#include <cassert>
#include <chrono>
#include <cstring>
#include <iostream>
#include <setjmp.h>
#include <string>
#include <thread>
#include <vector>

#ifdef _WIN32
// timeGetTime is unavailable if we use lean and mean
// #define WIN32_LEAN_AND_MEAN
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <objbase.h>
#include <windows.h>

// `small` is used as a type in `windows.h`
#undef small
#endif

#include "Audio/Audio.h"
#include "Config.h"
#include "Date.h"
#include "Economy/Economy.h"
#include "EditorController.h"
#include "Effects/EffectsManager.h"
#include "Entities/EntityManager.h"
#include "Entities/EntityTweener.h"
#include "Environment.h"
#include "Game.h"
#include "GameCommands/GameCommands.h"
#include "GameException.hpp"
#include "GameState.h"
#include "GameStateFlags.h"
#include "Graphics/Colour.h"
#include "Graphics/Gfx.h"
#include "Gui.h"
#include "Input.h"
#include "Input/Shortcuts.h"
#include "Interop/Hooks.h"
#include "Intro.h"
#include "Localisation/Formatting.h"
#include "Localisation/LanguageFiles.h"
#include "Localisation/Languages.h"
#include "Localisation/StringIds.h"
#include "Logging.h"
#include "Map/AnimationManager.h"
#include "Map/TileManager.h"
#include "Map/WaveManager.h"
#include "MessageManager.h"
#include "MultiPlayer.h"
#include "Network/Network.h"
#include "Objects/ObjectIndex.h"
#include "Objects/ObjectManager.h"
#include "OpenLoco.h"
#include "Random.h"
#include "S5/S5.h"
#include "ScenarioManager.h"
#include "ScenarioOptions.h"
#include "SceneManager.h"
#include "Title.h"
#include "Tutorial.h"
#include "Ui.h"
#include "Ui/ProgressBar.h"
#include "Ui/ToolTip.h"
#include "Ui/WindowManager.h"
#include "Vehicles/Vehicle.h"
#include "Vehicles/VehicleManager.h"
#include "ViewportManager.h"
#include "World/CompanyManager.h"
#include "World/IndustryManager.h"
#include "World/StationManager.h"
#include "World/TownManager.h"
#include "World/Company.h"
#include "World/Industry.h"
#include "World/Station.h"
#include "World/Town.h"
#ifdef OPENLOCO_FORCE_64BIT
#include "StructureLayoutLogger.h"
#endif
#include <OpenLoco/Core/Numerics.hpp>
#include <OpenLoco/Interop/Interop.hpp>
#include <OpenLoco/Platform/Crash.h>
#include <OpenLoco/Platform/Platform.h>
#include <OpenLoco/Utility/String.hpp>

#pragma warning(disable : 4611) // interaction between '_setjmp' and C++ object destruction is non - portable

using namespace OpenLoco::Interop;
using namespace OpenLoco::Ui;
using namespace OpenLoco::Input;
using namespace OpenLoco::Diagnostics;

namespace OpenLoco
{
    using Clock = std::chrono::high_resolution_clock;
    using Timepoint = Clock::time_point;

    static double _accumulator = 0.0;
    static Timepoint _lastUpdate = Clock::now();
    static CrashHandler::Handle _exHandler = nullptr;

    static uint16_t _time_since_last_tick = 0; // Was loco_global at 0x0050C19C
    static uint32_t _last_tick_time = 0; // Was loco_global at 0x0050C19E

    static int32_t _monthsSinceLastAutosave;

    static void autosaveReset();
    static void tickLogic(int32_t count);
    static void tickLogic();
    static void dateTick();

    std::string getVersionInfo()
    {
        return version;
    }

    // 0x004BE621
    [[noreturn]] void exitWithError(StringId titleStringId, StringId messageStringId)
    {
        char titleBuffer[256] = { 0 };
        char messageBuffer[256] = { 0 };
        StringManager::formatString(titleBuffer, 255, titleStringId);
        StringManager::formatString(messageBuffer, 255, messageStringId);
        Ui::showMessageBox(titleBuffer, messageBuffer);

        exitCleanly();
    }

    // 0x004BE65E
    [[noreturn]] void exitCleanly()
    {
        Audio::close();
        Audio::disposeDSound();
        Ui::disposeCursors();
        Localisation::unloadLanguageFile();

        auto tempFilePath = Environment::getPathNoWarning(Environment::PathId::_1tmp);
        if (fs::exists(tempFilePath))
        {
            auto path8 = tempFilePath.u8string();
            Logging::info("Removing temp file '{}'", path8.c_str());
            fs::remove(tempFilePath);
        }
        CrashHandler::shutdown(_exHandler);

        // Logging should be the last before terminating.
        Logging::shutdown();

        // SDL_Quit();
        exit(0);
    }

    // 0x00441400
    static void startupChecks()
    {
        const auto& config = Config::get();
        if (!config.allowMultipleInstances && !Platform::lockSingleInstance())
        {
            exitWithError(StringIds::game_init_failure, StringIds::loco_already_running);
        }

        // Originally the game would check that all the game
        // files exist are some have the correct checksum. We
        // do not need to do this anymore, the game should work
        // with g1 alone and some objects?
    }

    // 0x004C57C0
    void initialiseViewports()
    {
        Ui::Windows::MapToolTip::reset();

        Colours::initColourMap();
        Ui::WindowManager::init();
        Ui::ViewportManager::init();

        Input::init();
        Input::initMouse();

        // tooltip-related
        Ui::ToolTip::set_52336E(false);

        Ui::Windows::TextInput::cancel();

        // TODO Move this to a more generic, initialise game state function when
        //      we have one hooked / implemented.
        autosaveReset();
    }

    static void initialise()
    {
        Logging::info("INIT: Starting game initialization...");
        
        Logging::info("INIT: Getting platform time...");
        _last_tick_time = Platform::getTime();
        Logging::info("INIT: Platform time obtained successfully");

        Logging::info("INIT: Seeding random number generator...");
        std::srand(std::time(nullptr));
        Logging::info("INIT: Random number generator seeded");

        Logging::info("INIT: Allocating map elements...");
        World::TileManager::allocateMapElements();
        Logging::info("INIT: Map elements allocated successfully");
        
        Logging::info("INIT: Resolving environment paths...");
        Environment::resolvePaths();
        Logging::info("INIT: Environment paths resolved");
        
        Logging::info("INIT: Enumerating languages...");
        Localisation::enumerateLanguages();
        Logging::info("INIT: Languages enumerated successfully");
        
        Logging::info("INIT: Loading language file...");
        Localisation::loadLanguageFile();
        Logging::info("INIT: Language file loaded successfully");
        
        Logging::info("INIT: Running startup checks...");
        startupChecks();
        Logging::info("INIT: Startup checks completed");

        Logging::info("INIT: Loading G1 graphics (CRITICAL - likely 64-bit crash point)...");
        Gfx::loadG1();
        Logging::info("INIT: G1 graphics loaded successfully");
        
        Logging::info("INIT: Initializing graphics system...");
        Gfx::initialise();
        Logging::info("INIT: Graphics system initialized");

        Logging::info("INIT: Initializing UI system...");
        Ui::initialise();
        Logging::info("INIT: UI system initialized");
        
        Logging::info("INIT: Initializing cursors...");
        Ui::initialiseCursors();
        Logging::info("INIT: Cursors initialized");
        
        Logging::info("INIT: Initializing viewports...");
        initialiseViewports();
        Logging::info("INIT: Viewports initialized");
        
        Logging::info("INIT: Initializing GUI...");
        Gui::init();
        Logging::info("INIT: GUI initialized");

        Logging::info("INIT: Resetting message manager...");
        MessageManager::reset();
        Logging::info("INIT: Message manager reset");
        
        Logging::info("INIT: Resetting scenario...");
        Scenario::reset();
        Logging::info("INIT: Scenario reset");

        Logging::info("INIT: Loading object index...");
        ObjectManager::loadIndex();
        Logging::info("INIT: Object index loaded");
        
        Logging::info("INIT: Loading scenario index...");
        ScenarioManager::loadIndex();
        Logging::info("INIT: Scenario index loaded");

        Logging::info("INIT: Checking command line options...");
        const auto& cmdLineOptions = getCommandLineOptions();
        if (cmdLineOptions.action == CommandLineAction::intro)
        {
            Logging::info("INIT: Setting intro state to begin");
            Intro::state(Intro::State::begin);
        }
        else
        {
            Logging::info("INIT: Setting intro state to end");
            Intro::state(Intro::State::end);
        }

        Logging::info("INIT: Starting title sequence...");
        Title::start();
        Logging::info("INIT: Game initialization completed successfully!");
    }

    static void loadFile(const fs::path& path)
    {
        auto extension = path.extension().u8string();
        if (Utility::iequals(extension, S5::extensionSC5))
        {
            Scenario::loadAndStart(path);
        }
        else
        {
            S5::importSaveToGameState(path, S5::LoadFlags::none);
        }
    }

    static void loadFile(const std::string& path)
    {
        loadFile(fs::u8path(path));
    }

    static void launchGameFromCmdLineOptions()
    {
        const auto& cmdLineOptions = getCommandLineOptions();
        if (cmdLineOptions.action == CommandLineAction::host)
        {
            Network::openServer();
            loadFile(cmdLineOptions.path);
        }
        else if (cmdLineOptions.action == CommandLineAction::join)
        {
            if (cmdLineOptions.port)
            {
                Network::joinServer(cmdLineOptions.address, *cmdLineOptions.port);
            }
            else
            {
                Network::joinServer(cmdLineOptions.address);
            }
        }
        else if (!cmdLineOptions.path.empty())
        {
            loadFile(cmdLineOptions.path);
        }
    }

    void sub_431695(uint16_t var_F253A0)
    {
        GameCommands::setUpdatingCompanyId(CompanyManager::getControllingId());
        for (auto i = 0; i < var_F253A0; i++)
        {
            MessageManager::sub_428E47();
            WindowManager::dispatchUpdateAll();
        }

        Input::processKeyboardInput();
        WindowManager::update();
        Ui::handleInput();
        CompanyManager::updateOwnerStatus();
    }

    // This is called when the game requested to end the current tick early.
    // This can be caused by loading a new save game or exceptions.
    static void tickInterrupted()
    {
        EntityTweener::get().reset();
        Logging::info("Tick interrupted");
    }

    // 0x0046A794
    static void tick()
    {
        try
        {
            uint32_t time = Platform::getTime();
            _time_since_last_tick = (uint16_t)std::min(time - _last_tick_time, 500U);
            _last_tick_time = time;

            if (Tutorial::state() != Tutorial::State::none)
            {
                _time_since_last_tick = 31;
            }

            GameCommands::resetCommandNestLevel();
            Ui::update();

            {
                // Original called 0x00440DEC here which handled legacy cmd line options
                // like installing scenarios and handling multiplayer.

                Input::handleKeyboard();
                Input::processMouseMovement();
                Audio::updateSounds();

                Network::update();

                if (Intro::isActive())
                {
                    Intro::update();
                    if (!Intro::isActive())
                    {
                        launchGameFromCmdLineOptions();
                    }
                }
                else
                {
                    uint16_t numUpdates = std::clamp<uint16_t>(_time_since_last_tick / (uint16_t)31, 1, 3);
                    if (WindowManager::find(Ui::WindowType::multiplayer, 0) != nullptr)
                    {
                        numUpdates = 1;
                    }
                    if (SceneManager::isNetworked())
                    {
                        numUpdates = 1;
                    }
                    if (addr<0x00525324, int32_t>() == 1)
                    {
                        addr<0x00525324, int32_t>() = 0;
                        numUpdates = 1;
                    }
                    else
                    {
                        switch (Input::state())
                        {
                            case State::reset:
                            case State::normal:
                            case State::dropdownActive:
                                if (Input::hasFlag(Flags::viewportScrolling))
                                {
                                    Input::resetFlag(Flags::viewportScrolling);
                                    numUpdates = 1;
                                }
                                break;
                            case State::widgetPressed: break;
                            case State::positioningWindow: break;
                            case State::viewportRight: break;
                            case State::viewportLeft: break;
                            case State::scrollLeft: break;
                            case State::resizing: break;
                            case State::scrollRight: break;
                        }
                    }

                    Ui::WindowManager::setVehiclePreviewRotationFrame(Ui::WindowManager::getVehiclePreviewRotationFrame() + numUpdates);

                    if (SceneManager::isPaused())
                    {
                        numUpdates = 0;
                    }
                    uint16_t var_F253A0 = std::max<uint16_t>(1, numUpdates);
                    SceneManager::setSceneAge(std::min(0xFFFF, (int32_t)SceneManager::getSceneAge() + var_F253A0));
                    if (SceneManager::getGameSpeed() != GameSpeed::Normal)
                    {
                        numUpdates *= 3;
                        if (SceneManager::getGameSpeed() != GameSpeed::FastForward)
                        {
                            numUpdates *= 3;
                        }
                    }

                    // Catch up to server (usually after we have just joined the game)
                    auto numTicksBehind = Network::getServerTick() - ScenarioManager::getScenarioTicks();
                    if (numTicksBehind > 4)
                    {
                        numUpdates = 4;
                    }

                    tickLogic(numUpdates);

                    getGameState().var_014A++;
                    if (SceneManager::isEditorMode())
                    {
                        EditorController::tick();
                    }

                    Audio::playBackgroundMusic();

                    sub_431695(var_F253A0);
                }
            }
        }
        catch (GameException)
        {
            // Premature end of current tick; use a different message to indicate it's from C++ code
            tickInterrupted();
            return;
        }
    }

    static void tickLogic(int32_t count)
    {
        for (int32_t i = 0; i < count; i++)
        {
            tickLogic();
        }
    }

    static int8_t _loadErrorCode = 0; // Was loco_global at 0x0050C197
    static StringId _loadErrorMessage = 0; // Was loco_global at 0x0050C198

    // 0x0046ABCB
    static void tickLogic()
    {
        if (!Network::shouldProcessTick(ScenarioManager::getScenarioTicks() + 1))
        {
            return;
        }

        ScenarioManager::setScenarioTicks(ScenarioManager::getScenarioTicks() + 1);
        ScenarioManager::setScenarioTicks2(ScenarioManager::getScenarioTicks2() + 1);
        Network::processGameCommands(ScenarioManager::getScenarioTicks());

        recordTickStartPrng();
        World::TileManager::defragmentTilePeriodic();
        addr<0x00F25374, uint8_t>() = Scenario::getOptions().madeAnyChanges;
        dateTick();
        World::TileManager::update();
        World::WaveManager::update();
        TownManager::update();
        IndustryManager::update();
        VehicleManager::update();
        StationManager::update();
        EffectsManager::update();
        CompanyManager::update();
        World::AnimationManager::update();
        Audio::updateVehicleNoise();
        Audio::updateAmbientNoise();
        Title::update();

        Scenario::getOptions().madeAnyChanges = addr<0x00F25374, uint8_t>();
        if (_loadErrorCode != 0)
        {
            if (_loadErrorCode == -2)
            {
                StringId title = _loadErrorMessage;
                StringId message = StringIds::null;
                Ui::Windows::Error::open(title, message);
            }
            else
            {
                auto objectList = S5::getObjectErrorList();
                Ui::Windows::ObjectLoadError::open(objectList);
            }
            _loadErrorCode = 0;
        }
    }

    static void autosaveReset()
    {
        _monthsSinceLastAutosave = 0;
    }

    static void autosaveClean()
    {
        try
        {
            auto autosaveDirectory = Environment::getPath(Environment::PathId::autosave);
            if (fs::is_directory(autosaveDirectory))
            {
                std::vector<fs::path> autosaveFiles;

                // Collect all the autosave files
                for (auto& f : fs::directory_iterator(autosaveDirectory))
                {
                    if (f.is_regular_file())
                    {
                        auto& path = f.path();
                        auto filename = path.filename().u8string();
                        if (Utility::startsWith(filename, "autosave_") && Utility::endsWith(filename, S5::extensionSV5, true))
                        {
                            autosaveFiles.push_back(path);
                        }
                    }
                }

                auto amountToKeep = static_cast<size_t>(std::max(1, Config::get().autosaveAmount));
                if (autosaveFiles.size() > amountToKeep)
                {
                    // Sort them by name (which should correspond to date order)
                    std::sort(autosaveFiles.begin(), autosaveFiles.end());

                    // Delete excess files
                    auto numToDelete = autosaveFiles.size() - amountToKeep;
                    for (size_t i = 0; i < numToDelete; i++)
                    {
                        auto path8 = autosaveFiles[i].u8string();
                        Logging::info("Deleting old autosave: {}", path8.c_str());
                        fs::remove(autosaveFiles[i]);
                    }
                }
            }
        }
        catch (const std::exception& e)
        {
            Logging::error("Unable to clean autosaves: {}", e.what());
        }
    }

    static void autosave()
    {
        // Format filename
        auto time = std::time(nullptr);
        auto localTime = std::localtime(&time);
        char filename[64];
        snprintf(
            filename,
            sizeof(filename),
            "autosave_%04u-%02u-%02u_%02u-%02u-%02u%s",
            localTime->tm_year + 1900,
            localTime->tm_mon + 1,
            localTime->tm_mday,
            localTime->tm_hour,
            localTime->tm_min,
            localTime->tm_sec,
            S5::extensionSV5);

        try
        {
            auto autosaveDirectory = Environment::getPath(Environment::PathId::autosave);
            Environment::autoCreateDirectory(autosaveDirectory);

            auto autosaveFullPath = autosaveDirectory / filename;

            auto autosaveFullPath8 = autosaveFullPath.u8string();
            Logging::info("Autosaving game to {}", autosaveFullPath8.c_str());
            S5::exportGameStateToFile(autosaveFullPath, S5::SaveFlags::isAutosave | S5::SaveFlags::noWindowClose);
        }
        catch (const std::exception& e)
        {
            Logging::error("Unable to autosave game: {}", e.what());
        }
    }

    static void autosaveCheck()
    {
        _monthsSinceLastAutosave++;

        if (!SceneManager::isTitleMode())
        {
            auto freq = Config::get().autosaveFrequency;
            if (freq > 0 && _monthsSinceLastAutosave >= freq)
            {
                autosave();
                autosaveClean();
                autosaveReset();
            }
        }
    }

    // 0x004968C7
    static void dateTick()
    {
        if (Game::hasFlags(GameStateFlags::tileManagerLoaded) && !SceneManager::isEditorMode())
        {
            if (updateDayCounter())
            {
                StationManager::updateDaily();
                VehicleManager::updateDaily();
                IndustryManager::updateDaily();
                MessageManager::updateDaily();
                WindowManager::updateDaily();

                auto yesterday = calcDate(getCurrentDay() - 1);
                auto today = calcDate(getCurrentDay());
                setDate(today);
                Scenario::updateSnowLine(today.dayOfYear);
                Ui::Windows::TimePanel::invalidateFrame();

                if (today.month != yesterday.month)
                {
                    // End of every month
                    Scenario::getObjectiveProgress().monthsInChallenge++;
                    TownManager::updateMonthly();
                    IndustryManager::updateMonthly();
                    CompanyManager::updateMonthly1();
                    CompanyManager::updateMonthlyHeadquarters();
                    VehicleManager::updateMonthly();

                    if (today.year <= 2029)
                    {
                        Economy::updateMonthly();
                    }

                    // clang-format off
                    if (today.month == MonthId::january ||
                        today.month == MonthId::april ||
                        today.month == MonthId::july ||
                        today.month == MonthId::october)
                    // clang-format on
                    {
                        CompanyManager::updateQuarterly();
                    }

                    if (today.year != yesterday.year)
                    {
                        // End of every year
                        CompanyManager::updateYearly();
                        ObjectManager::updateDefaultLevelCrossingType();
                        ObjectManager::updateYearly2();
                        World::TileManager::updateYearly();
                    }

                    autosaveCheck();
                }

                CompanyManager::updateDaily();
            }
        }
    }

    static void tickWait()
    {
        // Idle loop for a 40 FPS
        do
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        } while (Platform::getTime() - _last_tick_time < Engine::UpdateRateInMs);
    }

    bool promptTickLoop(std::function<bool()> tickAction)
    {
        while (true)
        {
            _last_tick_time = Platform::getTime();
            _time_since_last_tick = 31;
            if (!Input::processMessages())
            {
                return false;
            }
            if (!tickAction())
            {
                break;
            }
            Ui::render();
            tickWait();
        }
        return true;
    }

    constexpr auto MaxUpdateTime = static_cast<double>(Engine::MaxTimeDeltaMs) / 1000.0;
    constexpr auto UpdateTime = static_cast<double>(Engine::UpdateRateInMs) / 1000.0;
    constexpr auto TimeScale = 1.0;

    static void variableUpdate()
    {
        auto& tweener = EntityTweener::get();

        const auto alpha = std::min<float>(_accumulator / UpdateTime, 1.0);

        while (_accumulator > UpdateTime)
        {
            tweener.preTick();

            tick();
            _accumulator -= UpdateTime;

            tweener.postTick();
        }

        tweener.tween(alpha);

        Ui::render();
    }

    static void fixedUpdate()
    {
        auto& tweener = EntityTweener::get();
        tweener.reset();

        if (_accumulator < UpdateTime)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        else
        {
            tick();
            _accumulator -= UpdateTime;

            Ui::render();
        }
    }

    static void update()
    {
        auto timeNow = Clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(timeNow - _lastUpdate).count() / 1'000'000.0;

        elapsed *= TimeScale;

        _accumulator = std::min(_accumulator + elapsed, MaxUpdateTime);
        _lastUpdate = timeNow;

        if (Config::get().uncapFPS)
        {
            variableUpdate();
        }
        else
        {
            fixedUpdate();
        }
    }

    // 0x00406386
    static void run()
    {
        Logging::info("MAIN LOOP: Starting main game loop...");
        
#ifdef _WIN32
        Logging::info("MAIN LOOP: Initializing COM (Windows)...");
        CoInitializeEx(nullptr, COINIT_MULTITHREADED);
        Logging::info("MAIN LOOP: COM initialized successfully");
#endif
        
        Logging::info("MAIN LOOP: Calling initialise() function (CRITICAL - likely crash point)...");
        initialise();
        Logging::info("MAIN LOOP: initialise() completed successfully");

        Logging::info("MAIN LOOP: Starting message processing loop...");
        int loopCount = 0;
        while (Input::processMessages())
        {
            loopCount++;
            if (loopCount % 100 == 1) // Log every 100th iteration to avoid spam
            {
                Logging::info("MAIN LOOP: Processing loop iteration {}", loopCount);
            }
            
            update();
            
            if (loopCount % 100 == 1)
            {
                Logging::info("MAIN LOOP: Update completed for iteration {}", loopCount);
            }
        }
        Logging::info("MAIN LOOP: Message processing loop ended after {} iterations", loopCount);

#ifdef _WIN32
        Logging::info("MAIN LOOP: Uninitializing COM (Windows)...");
        CoUninitialize();
        Logging::info("MAIN LOOP: COM uninitialized successfully");
#endif
        Logging::info("MAIN LOOP: Main game loop completed successfully");
    }

    void simulateGame(const fs::path& savePath, int32_t ticks)
    {
        Config::read();

        if (getCommandLineOptions().locomotionDataPath.has_value())
        {
            auto& cfg = Config::get();
            cfg.locoInstallPath = getCommandLineOptions().locomotionDataPath.value();
        }

        Environment::resolvePaths();

        try
        {
            initialise();
            loadFile(savePath);
        }
        catch (const std::exception& e)
        {
            Logging::error("Unable to simulate park: {}", e.what());
        }
        catch (const GameException i)
        {
            if (i != GameException::Interrupt)
            {
                Logging::error("Unable to simulate park!");
            }
            else
            {
                Logging::info("File loaded. Starting simulation.");
            }
        }
        tickLogic(ticks);
    }

    // 0x004078FE
    static void generateSystemStats()
    {
        Logging::info("SYSTEM STATS: Starting system statistics generation...");
        
        // Vanilla would actually query the system for this and would
        // also get the system computer name for default multiplayer name.
        // But there isn't much point nowadays to do this so lets just
        // set it to a large fixed value.
        // The value is only used by config to decide how many sounds
        // to have active at once.
        
#ifdef OPENLOCO_FORCE_64BIT
        // For 64-bit build, skip loco_global access that causes crash
        // This memory location doesn't exist in 64-bit address space
        Logging::info("SYSTEM STATS: Skipping loco_global memory access for 64-bit build");
        Logging::info("SYSTEM STATS: Total physical memory simulation bypassed");
#else
        static uint32_t _totalPhysicalMemory = 0; // Was loco_global at 0x0113E21C
        _totalPhysicalMemory = 0xFFFFFFFFU;
        Logging::info("SYSTEM STATS: Set total physical memory to 4GB (32-bit mode)");
#endif
        
        Logging::info("SYSTEM STATS: System statistics generation completed successfully");
    }

    // 0x00406D13
    static int main(const CommandLineOptions& options)
    {
        // Bootstrap the logging system.
        Logging::initialize(options.logLevels);

        // Always print the product name and version first.
        Logging::info("{}", OpenLoco::getVersionInfo());

#ifdef OPENLOCO_FORCE_64BIT
        // Log structure layouts for 64-bit debugging
        Debug::StructureLayoutLogger::initialize();
        LOG_SEPARATOR("OpenLoco 64-bit Structure Layout Analysis");
        LOG_NOTE("This log helps identify memory layout differences between 32-bit and 64-bit builds");
        
        LOG_SEPARATOR("Core Game Structures");
        LOG_STRUCT(Company);
        LOG_STRUCT(Industry);
        LOG_STRUCT(Station);
        LOG_STRUCT(Town);
        
        LOG_SEPARATOR("Company Structure Details - Finding the +4 Byte Difference");
        LOG_NOTE("Expected vs Actual offsets (32-bit vs 64-bit):");
        
        // Log first 20 members to find where +4 starts
        LOG_MEMBER(Company, name);
        LOG_MEMBER(Company, ownerName);
        LOG_MEMBER(Company, challengeFlags);
        LOG_MEMBER(Company, cash);
        LOG_MEMBER(Company, currentLoan);
        LOG_MEMBER(Company, updateCounter);
        LOG_MEMBER(Company, performanceIndex);
        LOG_MEMBER(Company, competitorId);
        LOG_MEMBER(Company, ownerEmotion);
        LOG_MEMBER(Company, mainColours);
        LOG_MEMBER(Company, customVehicleColoursSet);
        LOG_MEMBER(Company, vehicleColours);
        LOG_MEMBER(Company, headquartersZ);
        LOG_MEMBER(Company, headquartersX);
        LOG_MEMBER(Company, headquartersY);
        
        // The failing ones from static assertions
        LOG_MEMBER(Company, companyValueHistory);
        LOG_MEMBER(Company, vehicleProfit);
        LOG_MEMBER(Company, challengeProgress);
        LOG_MEMBER(Company, activeEmotions);
        
        LOG_SEPARATOR("Size Analysis");
        LOG_NOTE("Total size difference: 4 bytes (0x8FAC - 0x8FA8)");
        LOG_NOTE("All measured offsets are +4 bytes from expected");
        LOG_NOTE("This suggests a single size change early in the structure");
        
        Debug::StructureLayoutLogger::close();
        Logging::info("64-bit structure layout analysis saved to structure_layout_64bit.log");
#endif

        Environment::setLocale();

        auto ret = runCommandLineOnlyCommand(options);
        if (ret)
        {
            return *ret;
        }

        setCommandLineOptions(options);

        if (!OpenLoco::Platform::isRunningInWine())
        {
            CrashHandler::AppInfo appInfo;
            appInfo.name = "OpenLoco";
            appInfo.version = getVersionInfo();

            _exHandler = CrashHandler::init(appInfo);
        }
        else
        {
            Logging::warn("Detected wine, not installing crash handler as it doesn't provide useful data. Consider using native builds of OpenLoco instead.");
        }

        try
        {
            Logging::info("Starting OpenLoco initialization sequence...");
            
            Logging::info("Step 1: Initializing input shortcuts...");
            Input::Shortcuts::initialize();
            Logging::info("Step 1: Input shortcuts initialized successfully");

            Logging::info("Step 2: Reading configuration...");
            const auto& cfg = Config::read();
            Logging::info("Step 2: Configuration read successfully");
            
            Logging::info("Step 3: Resolving environment paths...");
            Environment::resolvePaths();
            Logging::info("Step 3: Environment paths resolved successfully");

            Logging::info("Step 4: Creating UI window (CRITICAL STEP - likely crash point)...");
            Ui::createWindow(cfg.display);
            Logging::info("Step 4: UI window created successfully");
            
            Logging::info("Step 5: Generating system statistics...");
            generateSystemStats();
            Logging::info("Step 5: System statistics generated successfully");
            
            Logging::info("Step 6: Initializing audio/DirectSound...");
            Audio::initialiseDSound();
            Logging::info("Step 6: Audio initialized successfully");
            
            Logging::info("Step 7: Starting main game loop...");
            run();
            Logging::info("Step 7: Main game loop completed");
            
            Logging::info("All steps completed successfully, cleaning up...");
            exitCleanly();
        }
        catch (const std::exception& e)
        {
            Logging::error("CRASH DEBUG: Caught std::exception during initialization: {}", e.what());
            Logging::error("CRASH DEBUG: This indicates the crash occurred during one of the initialization steps");
            try {
                Ui::showMessageBox("OpenLoco 64-bit Debug", std::string("Exception during initialization: ") + e.what());
            } catch (...) {
                std::cerr << "FATAL: Exception during initialization and cannot show message box: " << e.what() << std::endl;
            }
            exitCleanly();
        }
        catch (...)
        {
            Logging::error("CRASH DEBUG: Caught unknown exception during initialization");
            Logging::error("CRASH DEBUG: This is likely a segfault or similar low-level crash");
            try {
                Ui::showMessageBox("OpenLoco 64-bit Debug", "Unknown exception during initialization - likely segfault");
            } catch (...) {
                std::cerr << "FATAL: Unknown exception during initialization and cannot show message box" << std::endl;
            }
            exitCleanly();
        }
    }

    int main(std::vector<std::string>&& argv)
    {
        auto options = parseCommandLine(std::move(argv));
        if (options)
        {
            return main(*options);
        }
        else
        {
            return 1;
        }
    }
}
