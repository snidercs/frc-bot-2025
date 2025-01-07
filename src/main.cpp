#include <iostream>
#include <memory>
#include <regex>
#include <string>

#include <frc/Filesystem.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/controller/RamseteController.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/trajectory/TrajectoryGenerator.h>

#include <cameraserver/CameraServer.h>
#if 0
#    include <opencv2/core/core.hpp>
#    include <opencv2/core/types.hpp>
#    include <opencv2/imgproc/imgproc.hpp>
#endif

#include "snider/padmode.hpp"

#include "include/config.hpp"
#include "include/engine.hpp"
#include "include/normalisablerange.hpp"
#include "include/parameters.hpp"
#include "include/scripting.hpp"

#include "include/robot.hpp"
#include "sol/table.hpp"

#define SIM_CAMERA_DISABLED 1

namespace lua {
extern void bind_gamepad (frc::XboxController*);
}

namespace detail {

// make a pose2d from a lua table.
frc::Pose2d makePose2d (sol::table tbl) {
    return frc::Pose2d (
        units::meter_t (tbl[1].get<double>()),
        units::meter_t (tbl[2].get<double>()),
        frc::Rotation2d (units::radian_t (tbl[3].get<double>())));
}

// make a trajectory config from a lua table.
frc::TrajectoryConfig makeTrajectoryConfig (sol::table tbl) {
    return frc::TrajectoryConfig (
        units::meters_per_second_t (tbl[1].get<double>()),
        units::meters_per_second_squared_t (tbl[2].get<double>()));
}

static bool trajectoryIsReverse (std::string_view symbol, bool otherwise = true) {
    sol::function trajectory = lua::state()["config"]["trajectory"];
    sol::table tbl           = trajectory (symbol);
    return tbl.get_or ("reverse", otherwise);
}

// compose a trajectory from lua configuration.
frc::Trajectory makeTrajectory (std::string_view symbol) {
    sol::function trajectory = lua::state()["config"]["trajectory"];
    sol::table tbl           = trajectory (symbol);
    return frc::TrajectoryGenerator::GenerateTrajectory (
        makePose2d (tbl["start"]),
        {},
        makePose2d (tbl["stop"]),
        makeTrajectoryConfig (tbl["config"]));
}

static void displayBanner() {
    // display engine and bot info.
    lua::print_version();
    std::clog << "Engine running at "
              << config::number ("engine", "period")
              << " ms" << std::endl;
    std::clog.flush();
    std::cout.flush();
    std::cerr.flush();
}

static std::string findLuaDir() {
    return lua::search_directory();
}

/** Instantiate the robot. This could return a bot in error state. Be sure to
    check with `Robot::have_error()` and `Robot::error()`.
*/
static EnginePtr instantiateRobot (std::string_view botfile) {
    auto& L = lua::state();
    std::filesystem::path path (findLuaDir());
    path /= botfile;
    path.make_preferred();
    return Engine::instantiate (L.lua_state(), path.string());
}

} // namespace detail

//==============================================================================
/** Choose which bot file to run in test mode. */
class TestProgramChooser final {
public:
    TestProgramChooser() {
        namespace fs = std::filesystem;
        using Iter   = fs::directory_iterator;
        const Iter end;
        std::vector<std::string> files;

        try {
            auto path = fs::path (detail::findLuaDir());
            for (Iter iter { path }; iter != end; iter++) {
                if (! fs::is_regular_file (*iter))
                    continue;

                const std::string ext = iter->path().extension().string();
                if (std::regex_match (ext, std::regex ("\\.(?:bot)")))
                    files.push_back ((*iter).path().filename().string());
            }

            chooser.SetDefaultOption (defaultTest, defaultTest);
            for (const auto& f : files)
                chooser.AddOption (f, f);
            frc::SmartDashboard::PutData ("Test Program", &chooser);
        } catch (...) {
            _valid = false;
        }
    }

    std::string get() const { return valid() ? chooser.GetSelected() : defaultTest; }
    bool valid() const noexcept { return _valid; }

private:
    frc::SendableChooser<std::string> chooser;
    bool _valid { true };
    const std::string defaultTest { "teleop.bot" };
};

//==============================================================================
/** Auto mode settings to use during operation. see config.lua */
struct AutoModeInfo {
    std::string name;
    bool reverse { true };
    frc::Trajectory trajectory;
};

/** Choose which auto mode to use in Autonomous. */
class AutoModeChooser final {
public:
    AutoModeChooser() {
        // setup the dashboard chooser.
        for (const auto& m : config::trajectory_names())
            chooser.AddOption (m, m);
        chooser.SetDefaultOption (defaultMatchStart, defaultMatchStart);
        frc::SmartDashboard::PutData ("Auto Mode", &chooser);
    }

    /**
     * Returns an AutoModeInfo representation of this chooser.
     * @returns the AutoModeInfo structure
    */
    AutoModeInfo info() const {
        AutoModeInfo info;
        info.name       = get();
        info.reverse    = detail::trajectoryIsReverse (info.name, info.reverse);
        info.trajectory = detail::makeTrajectory (info.name);
        return info;
    }

    /** Returns the auto mode to use by key identifier. */
    std::string get() const {
        auto val = chooser.GetSelected();
        if (val.empty())
            val = defaultMatchStart;
        return val;
    }

    /**
     * Returns true if the bot should move in reverse.
     */
    constexpr bool reverse() const noexcept { return _reverse; }

private:
    frc::SendableChooser<std::string> chooser;
    const std::string defaultMatchStart { config::match_start_position() };
    bool _reverse { true };
};

//==============================================================================
class RobotMain : public frc::TimedRobot {
public:
    RobotMain()
        : frc::TimedRobot (units::millisecond_t (
            config::number ("engine", "period"))) {
        // bind instances to Lua
        Parameters::bind (&params);
        Drivetrain::bind (&drivetrain);
        lua::bind_gamepad (&gamepad);

        detail::displayBanner();
    }

    ~RobotMain() {
        engine.reset();

        // release instances from lua
        Parameters::bind (nullptr);
        Drivetrain::bind (nullptr);
        lua::bind_gamepad (nullptr);
    }

    void RobotInit() override {
        testProgram = std::make_unique<TestProgramChooser>();
        autoMode    = std::make_unique<AutoModeChooser>();
        reloadTrajectory();
        collectGarbage();

// #ifndef RUNNING_FRC_TESTS
#if 0
        std::thread vision (visionThread);
        vision.detach();
#endif
    }

    /** Called periodically no matter the mode. This RUNS AFTER the mode 
        specific periodic functions, but before LiveWindow and SmartDashboard 
        integrated updating.
     */
    void RobotPeriodic() override {
        drivetrain.postProcess();
    }

    void AutonomousInit() override {
        reloadTrajectory();

        timer.Restart();
        drivetrain.resetOdometry (autoInfo.trajectory.InitialPose());
        collectGarbage();
    }

    void AutonomousPeriodic() override {
    }

    void AutonomousExit() override {
        collectGarbage();
    }

    //==========================================================================
    void TeleopInit() override {
        protectedLuaCalls = false;
        if (! safeLoadEngine ("teleop.bot"))
            return;
        luaPrepare();
    }
    void TeleopPeriodic() override { luaPeriodic(); }
    void TeleopExit() override { luaExit(); }

    //==========================================================================
    void DisabledInit() override { collectGarbage(); }
    void DisabledPeriodic() override { driveDisabled(); }
    void DisabledExit() override { collectGarbage(); }

    //==========================================================================
    void TestInit() override {
        protectedLuaCalls   = true;
        luaErrorEncountered = false;
        if (! safeLoadEngine (testProgram->get()))
            return;
        luaPrepare();
    }

    void TestPeriodic() override { luaPeriodic(); }
    void TestExit() override { luaExit(); }

    //==========================================================================
    void SimulationInit() override {
        drivetrain.initializeSimulation();
    }

    void SimulationPeriodic() override {
        drivetrain.updateSimulation();
    }

private:
    EnginePtr engine;
    Parameters params;

    frc::XboxController gamepad { config::port ("gamepad") };
    Drivetrain drivetrain;

    bool gamepadConnected = false; // Track controller connection state.

    std::unique_ptr<TestProgramChooser> testProgram;

    std::unique_ptr<AutoModeChooser> autoMode;
    AutoModeInfo autoInfo;
    frc::RamseteController ramsete;
    frc::Timer timer;
    bool hasStartedMoving = false; // track auto bot movement started

    bool luaErrorEncountered = false;
    bool protectedLuaCalls   = false;
    //==========================================================================
    void collectGarbage() {
        lua::state().collect_garbage();
    }

    // Load a bot program by name e.g. teleop.bot or engine_test.bot
    // will throw a runtime error if the loaded engine is in error state.
    void loadEngine (std::string_view bot) {
        auto newEngine = detail::instantiateRobot (bot);
        if (newEngine != nullptr) {
            if (newEngine->have_error()) {
                throw std::runtime_error (newEngine->error());
            }

            newEngine->init();
            std::swap (engine, newEngine);
            newEngine.reset();
            std::clog << "[bot] program loaded: " << bot << std::endl;
        } else {
            throw std::runtime_error ("Failed to instantiate Lua engine");
        }
    }

    bool safeLoadEngine (std::string_view bot) {
        luaErrorEncountered = false;

        try {
            loadEngine (bot);
        } catch (const std::exception& e) {
            std::cerr << "[lua] exception: " << e.what() << std::endl;
            std::flush (std::cerr);
            luaErrorEncountered = true;
        }

        return ! luaErrorEncountered;
    }

    // reloads/resets the currently selected AutoMode info.
    void reloadTrajectory() {
        try {
            autoInfo         = autoMode->info();
            hasStartedMoving = false;
        } catch (const std::exception& e) {
            std::cerr
                << "[bot] error: lua trajectory could not be parsed." << std::endl
                << "[bot] what: " << e.what() << std::endl
                << "[bot] loading fallback trajectory" << std::endl;
            autoInfo            = {};
            autoInfo.name       = "Fallback";
            autoInfo.trajectory = frc::TrajectoryGenerator::GenerateTrajectory (
                frc::Pose2d { 2_m, 2_m, 0_rad },
                {},
                frc::Pose2d { 2.5_m, 2_m, 0_rad },
                frc::TrajectoryConfig (0.75_mps, 2_mps_sq));
        }
    }

    //==========================================================================
    void luaLogErrorIfPresent() {
        if (! luaErrorEncountered)
            return;
        const auto msg = engine->have_error() ? engine->error() : std::string ("unknown Lua error");
        std::cerr << "[lua] error: " << msg << std::endl;
    }

    void luaPrepare() {

        if (! luaErrorEncountered) {
            luaErrorEncountered = ! engine->prepare();
            luaLogErrorIfPresent();
        }

        collectGarbage();
    }

    void luaPeriodic() {
        if (! checkControllerConnection() || luaErrorEncountered) {
            driveDisabled();
        } else {
            processParameters();

            if (! protectedLuaCalls) {
                engine->run();
            } else if (! luaErrorEncountered) {
                luaErrorEncountered = ! engine->safe_run();
                luaLogErrorIfPresent();
            }
        }
    }

    void luaExit() {
        if (! luaErrorEncountered) {
            luaErrorEncountered = ! engine->cleanup();
            luaLogErrorIfPresent();
        }

        collectGarbage();
    }

    //==========================================================================
    void driveDisabled() {
        drivetrain.drive(MetersPerSecond(0), MetersPerSecond(0), RadiansPerSecond(0));
    }

    // updates controller connection status, returns true if currently connected
    bool checkControllerConnection() {
        const bool wasConnected = gamepadConnected;
        gamepadConnected        = gamepad.IsConnected();
        if (gamepadConnected != wasConnected) {
            // noop. connection status changed.
        }
        return gamepadConnected;
    }

    // Filter parameter values before driving the bot.
    void processParameters() {
        Parameters::Context ctx;

        if (! gamepadConnected)
            return;

        // Copy axis values.
        for (int i = std::min (gamepad.GetAxisCount(), (int) Parameters::MaxAxes); --i >= 0;) {
            ctx.axis[i] = gamepad.GetRawAxis (i);
            // needed? gamepad.GetAxisType()
        }

        // Copy dpad info.
        ctx.povs[0] = gamepad.GetPOVCount() <= 0 ? 0 : gamepad.GetPOV (0);

        // copy button values. button indexing in FRC starts at 1 for some reason.
        for (int i = std::min (gamepad.GetButtonCount(), (int) Parameters::MaxButtons); --i >= 0;) {
            ctx.buttons[i] = gamepad.GetRawButton (i + 1);
        }

        params.process (ctx);
    }

    static void visionThread() {
#if SIM_CAMERA_DISABLED
        if (RobotBase::IsSimulation())
            return;
#endif
        const auto cameraName = "Camera 1";
        const auto width      = 640;
        const auto height     = 360;
        const auto fps        = 20;

        // Get the USB camera from CameraServer
        cs::UsbCamera camera = frc::CameraServer::StartAutomaticCapture (cameraName, 0);
        camera.SetExposureAuto();
        camera.SetWhiteBalanceAuto();

        // Set the resolution
        camera.SetResolution (width, height);
        camera.SetFPS (fps);

#if 0
        // Get a CvSink. This will capture Mats from the Camera
        cs::CvSink cvSink = frc::CameraServer::GetVideo();
        // Setup a CvSource. This will send images back to the Dashboard
        cs::CvSource outputStream =
            frc::CameraServer::PutVideo (cameraName, width, height);

        // Mats are very memory expensive. Lets reuse this Mat.
        cv::Mat mat;

        while (true) {
            // Tell the CvSink to grab a frame from the camera and
            // put it
            // in the source mat.  If there is an error notify the
            // output.
            if (cvSink.GrabFrame (mat) == 0) {
                // Send the output the error.
                outputStream.NotifyError (cvSink.GetError());
                // skip the rest of the current iteration
                continue;
            }
#    if 0
            // Put a rectangle on the image
            rectangle (mat, cv::Point (100, 100), 
                            cv::Point (400, 400), 
                            cv::Scalar (255, 255, 255), 5);
            // Give the output stream a new image to display
            outputStream.PutFrame (mat);
#    endif
        }
#endif
    }
};

#ifndef RUNNING_FRC_TESTS
/** This is not ideal, but frc::StartRobot instantiates a singleton version
    of Robot main with no explicit shutdown.  Our lua engine must exist before
    and after the robot's ctor and dtor. Having lifecylce at the global scope
    helps avoid crashes when the app exits.
*/
static lua::Lifecycle luaEngine;

int main() {
    if (! lua::bootstrap())
        throw std::runtime_error ("lua engine could not be bootstrapped");
    return frc::StartRobot<RobotMain>();
}
#else
frc::TimedRobot* instantiate_robot() {
    auto bot = new RobotMain();
    bot->RobotInit();
    return bot;
}
#endif
