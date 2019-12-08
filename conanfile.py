
from conans import ConanFile, CMake, tools

class ViterbiSnakes(ConanFile):
    settings = "os", "compiler", "build_type", "arch"
    generators = "cmake_find_package"

    def requirements(self):
        self.requires('eigen/3.3.7@conan/stable')

    def build_requirements(self):
        self.build_requires('gtest/1.8.1@bincrafters/stable')

    def imports(self):
        self.copy("*.dll", dst="bin", src="bin")
        self.copy("*.dylib*", dst="lib", src="lib")
        self.copy('*.so*', dst='lib', src='lib')