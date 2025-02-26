project ('frc-bot-2025', ['cpp', 'c'], 
    version : '0.0.1',
    meson_version : '>= 1.4.0',
    license : 'BSD',
    default_options: [
        'b_ndebug=if-release',
        'buildtype=release',
        'cpp_std=c++20',
    ])

fs = import ('fs')
cmake = import('cmake')

c = meson.get_compiler ('c')
cpp = meson.get_compiler ('cpp')

boost_dep = dependency ('boost', required : false)

# TODO: these should be dynamic and/or option-ized
frc_is_sim = not meson.is_cross_build()
frc_platform = frc_is_sim ? 'linuxx86-64' : 'linuxathena'
frc_buildtype = 'release'
frc_libdir = meson.project_source_root() / 'build/install/frcUserProgram/@0@/@1@/lib'.format(frc_platform, frc_buildtype)

subdir ('meson/allwpilib')

luabot_proj = subproject ('luabot', default_options: [
   'test=@0@'.format(frc_is_sim ? 'enabled' : 'disabled'),
    'frc-libs=@0@'.format(frc_libdir)
])
luabot_dep = luabot_proj.get_variable ('luabot_dep')

# Phoenix 6
phoenix6_libs = []
phoenix6_sonames = []

if frc_is_sim
    phoenix6_sonames = '''
        CTRE_Phoenix6_WPISim
        CTRE_SimVictorSPX
        CTRE_PhoenixTools_Sim
        CTRE_SimProPigeon2
        CTRE_SimPigeonIMU
        CTRE_SimProCANdi
        CTRE_SimProCANcoder
        CTRE_SimProTalonFXS
        CTRE_SimProCANrange
        CTRE_SimProTalonFX
        CTRE_SimTalonSRX
        CTRE_SimCANCoder
    '''.split()
else
    phoenix6_sonames = '''
        CTRE_Phoenix6_WPI
        CTRE_PhoenixTools
    '''.split()
endif

foreach lib: phoenix6_sonames
    phoenix6_libs += cpp.find_library (lib, dirs: [ frc_libdir ], required: true)
endforeach

phoenix6_dep = declare_dependency (
    include_directories: fs.expanduser ('~/SDKs/include'),
    dependencies: phoenix6_libs
)

wpilib_deps += phoenix6_dep

# Firmware
subdir ('src')

# Tests
if boost_dep.found()
    subdir ('tests')
endif

summary ('Build type', frc_buildtype)
summary ('Simulator', frc_is_sim)
summary ('Tests', boost_dep.found())
