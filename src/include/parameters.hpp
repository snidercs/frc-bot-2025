#pragma once

#include <cstring> // for memcpy, memset

#include "snider/padmode.hpp"

/** Parameter state. e.g. Raw controller value storage and filtering.
 
    It is this object's job to process data coming from the FRC system.  Think 
    of it  as a realtime state of all the data points needed to control the bot.  
    e.g. it contains all the information needed to send messages to, change 
    voltages of, etc etc. It gets updated in real-time in the frc callbacks. And 
    vise versa, the FRC classes call functions on this class to retrieve info 
    when calculating voltages etc etc etc.

    General Usage: Stick a Parameters object in your main class.  Then inside the
    periodic functions instantiate a Parameters::Context, copy raw joystick and
    button values, then pass to Parameters::process().
*/
class Parameters final {
public:
    using PadMode = snider::PadMode;

    Parameters()  = default;
    ~Parameters() = default;

    /** call once to bind to root Lua context. */
    static bool bind (Parameters* self);

    Parameters (const Parameters& o) { operator= (o); }
    Parameters& operator= (const Parameters& o) {
        values     = o.values;
        lastValues = o.lastValues;
        return *this;
    }

    Parameters (Parameters&& o) { operator= (std::forward<Parameters> (o)); }
    Parameters& operator= (Parameters&& o) {
        values     = std::move (o.values);
        lastValues = std::move (o.lastValues);
        return *this;
    }

    /** Dead zone threshold */
    static constexpr double DeadZone = 0.1;

    enum : int {
        MaxAxes    = 32, ///> Max number of axes supported.
        MaxPOVs    = 1,  ///> Max number of dpads supported.
        MaxButtons = 16  ///> Max number of buttons supported.
    };

    enum Indexes : int {
        LeftStickX   = 0, ///> Index of Left stick X
        LeftStickY   = 1, ///> Index of Left stick Y
        TriggerLeft  = 2, ///> Index of Trigger left
        TriggerRight = 3, ///> Index of Trigger right
        RightStickX  = 4, ///> Index of Right stick X
        RightStickY  = 5  ///> Index of Right stick Y
    };

    enum : int {
        ButtonA           = 0, ///> Index of Button A
        ButtonB           = 1, ///> Index of Button B
        ButtonX           = 2, ///> Index of Button X
        ButtonY           = 3, ///> Index of Button Y
        ButtonLeftBumper  = 4, ///> Index of Right Bumper
        ButtonRightBumper = 5, ///> Index of Left Bumper

        ButtonBack  = 6, ///> Index of Back Button
        ButtonStart = 7, ///> Index of Start Button
        ButtonHome  = 8, ///> Index of Home Button
        ButtonL3    = 9, ///> Index of L3 Button
        ButtonR3    = 10 ///> Index of R3 Button
    };
    /** A context in which controller values get processed runs. The context 
        holds a cache of raw values read from the Xbox, or other, controller.
     */
    class Context {
    public:
        Context() { reset(); }
        ~Context() = default;

        /** Copy ctor and operator. */
        Context (const Context& o) { *this = o; }
        Context& operator= (const Context& o) noexcept {
            // This literally means memory copy, and that's exactly what it
            // does... copy raw memory from one address to another and how many
            // bytes.
            memcpy (axis, o.axis, MaxAxes * sizeof (double));
            memcpy (povs, o.povs, MaxPOVs * sizeof (int));
            memcpy (buttons, o.buttons, MaxButtons * sizeof (bool));
            return *this;
        }

        /** Storage for axis data (the thumb sticks and triggers)
            
            This is a plain C-style array with a fixed size.  It is a direct
            allocation in RAM on the stack.  Java cannot do this that I know of,
            which is to have raw, direct access to memory... lighthing fast to 
            get/set values from these types of arrays.
        */
        double axis[MaxAxes] = { 0 };

        /** POVs storage. */
        int povs[MaxPOVs] = { 0 };

        /** Buttons storage. */
        bool buttons[MaxButtons] = { 0 };

        /** Reset to default values */
        inline void reset() noexcept {
            memset (buttons, 0, MaxButtons * sizeof (bool));
            memset (axis, 0, MaxAxes * sizeof (double));
            memset (povs, 0, MaxPOVs * sizeof (int));
        }
    };

    /** Reset all values to default. */
    void reset() noexcept;

    /** Returns the current gamepad mode. */
    constexpr auto getPadMode() const noexcept { return padMode; }

    /** Returns the pad mode as a string. */
    std::string getPadModeString() const noexcept {
        return std::to_string (padMode);
    }

    /** Change the gamepad mode.
     
        @param newMode The new gamepad mode to use when retrieving parameter
                       values.
    */
    void setPadMode (snider::PadMode newMode) noexcept {
        if (newMode == padMode)
            return;
        padMode = newMode;
    }

    /** Process the context, update values, prepare for replies to bot.
        
        @param context Prepared by main, called from the realtime function.
     */
    void process (const Context& context) noexcept;

    /** Returns the axis value of the given axis */
    double getAxisValue (int index) const noexcept {
        return values.axis[index];
    }

    /** Returns the processed left stick X value. */
    double getLeftStickX() const noexcept { return getAxisValue (LeftStickX); }

    /** Returns the processed left stick Y value. */
    double getLeftStickY() const noexcept { return getAxisValue (LeftStickY); }

    /** Returns the processed right stick X value. */
    double getRightStickX() const noexcept { return getAxisValue (RightStickX); }

    /** Returns the processed right stick Y value. */
    double getRightStickY() const noexcept { return getAxisValue (RightStickY); }

    /** Returns the processed left trigger value. */
    double getTriggerLeft() const noexcept { return getAxisValue (TriggerLeft); }

    /** Returns the processed right trigger value. */
    double getTriggerRight() const noexcept { return getAxisValue (TriggerRight); }

    /** Return processed passed value. */
    bool getButtonValue (int button) const noexcept { return values.buttons[button]; }

    /** Returns the processed A button value. */
    bool getAButton() const noexcept { return getButtonValue (ButtonA); }

    /** Returns the processed B button value. */
    bool getBButton() const noexcept { return getButtonValue (ButtonB); }

    /** Returns the processed X button value. */
    bool getXButton() const noexcept { return getButtonValue (ButtonX); }

    /** Returns the processed Y button value. */
    bool getYButton() const noexcept { return getButtonValue (ButtonY); }

    /** Returns the processed right bumper button value. */
    bool getRightBumperButton() const noexcept { return getButtonValue (ButtonRightBumper); }

    /** Returns the processed left bumper button value. */
    bool getLeftBumperButton() const noexcept { return getButtonValue (ButtonLeftBumper); }

    /** Returns the processed back button value. */
    bool getBackButton() const noexcept { return getButtonValue (ButtonBack); }
    /** Returns the processed start button value. */

    bool getStartButton() const noexcept { return getButtonValue (ButtonStart); }

    /** Returns the processed home button value. */
    bool getHomeButton() const noexcept { return getButtonValue (ButtonHome); }

    /** Returns the processed l3 button value. */
    bool getL3Button() const noexcept { return getButtonValue (ButtonL3); }

    /** Returns the processed r3 button value. */
    bool getR3Button() const noexcept { return getButtonValue (ButtonR3); }

    /** Return processed passed value. */
    int getPOVValue (int index) const noexcept { return values.povs[index]; }

    //==========================================================================
    /** Get the normalized speed value.
        
        @returns A speed value in the range of -1.0 to 1.0
    */
    double getSpeed() const noexcept {
        double val = 0.0;

        switch (padMode) {
            case PadMode::Standard:
                val = getLeftStickY();
                break;
        }

        return val;
    }

    /** Get the normalized angular speed value.
        
        @returns A speed value in the range of -1.0 to 1.0
    */
    double getAngularSpeed() const noexcept {
        double val = 0.0;

        switch (padMode) {
            case PadMode::Standard:
                val = getRightStickX();
                break;
        }

        return val;
    }

    /** Get the brake value. (0.0 to 1.0) */
    double getBrake() const noexcept {
        double val = 0.0;

        switch (padMode) {
            case PadMode::Standard:
                val = getTriggerLeft();
                break;
        }

        return val;
    }

private:
    Context values;
    Context lastValues;
    PadMode padMode { PadMode::Standard };
};
