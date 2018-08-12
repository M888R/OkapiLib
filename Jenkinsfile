job('example') {
    steps {
        cmake {
            cmakeInstallation('InSearchPath')
            generator('CodeBlocks - Unix Makefiles')
            cleanBuild()
            sourceDir('src')
            buildDir('cmake-build-debug')
            buildToolStep {
                useCmake()
            }
        }
    }
}
