#include <util/validators.hpp>

#include <filesystem>

namespace lsd_slam {

    void ExistingFile(const std::string &file) {
        if (std::filesystem::exists(file)) {
            throw std::runtime_error{file + " doesn't exist"};
        }
    }
}
