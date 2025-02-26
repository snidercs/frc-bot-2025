if frc_is_sim
    # Check if wpilib is installed, if not build it as a subproject.
    wpilibc_dep  = dependency ('wpilibc', method: 'cmake', required: false)
    if wpilibc_dep.found()
        hal_dep      = dependency ('hal', method: 'cmake', required: true)
        ntcore_dep   = dependency ('ntcore', method: 'cmake', required: true)
        wpiutil_dep  = dependency ('wpiutil', method: 'cmake', required: true)
        wpimath_dep  = dependency ('wpimath', method: 'cmake', required: true)
        apriltag_dep = dependency ('apriltag', method: 'cmake', required: true)
        cameraserver_dep = dependency ('cameraserver', method: 'cmake', required: true)
        cscore_dep   = dependency ('cscore', method: 'cmake', required: true)
        newcommands_dep = dependency ('wpilibnewcommands', method: 'cmake', required: true)
    else
        wpilib_opts = cmake.subproject_options()
        wpilib_opts.add_cmake_defines ({
            'WITH_JAVA': false,
            'WITH_DOCS': false,
            'WITH_TESTS': false
        })

        allwpilib = cmake.subproject ('allwpilib', required: true, options: wpilib_opts)
        # list cmake targets
        # message('CMake targets:\n - ' + '\n - '.join(allwpilib.target_list()))

        wpiutil_dep  = allwpilib.dependency ('wpiutil')
        apriltag_dep = allwpilib.dependency ('apriltag')
        cameraserver_dep = allwpilib.dependency ('cameraserver')
        cscore_dep   = allwpilib.dependency ('cscore')
        hal_dep = allwpilib.dependency('hal')
        ntcore_dep = allwpilib.dependency('ntcore')
        hal_dep = allwpilib.dependency('hal')
        wpilibc_dep = allwpilib.dependency('wpilibc')
        wpimath_dep = allwpilib.dependency('wpimath')
        newcommands_dep = allwpilib.dependency('wpilibNewCommands')
    endif

    wpilib_deps = [ wpilibc_dep,
        wpiutil_dep,
        apriltag_dep,
        wpimath_dep,
        ntcore_dep, 
        hal_dep, 
        newcommands_dep
    ]
else
    wpilib_sonames = '''
        cameraserver
        cscore
        wpimathjni
        wpiHal
        wpiutil
        ntcorejni
        wpiHaljni
        ntcore
        wpilibc
        wpimath
        cscorejni
        wpinet
        wpilibNewCommands
        apriltagjni
        wpinetjni
        apriltag
        wpiutiljni
        opencv_java4100
        pthread
    '''.split()
    wpilib_deps = []

    foreach lib: wpilib_sonames
        wpilib_deps += cpp.find_library (lib, dirs: [ frc_libdir ], required: true)
    endforeach
endif
