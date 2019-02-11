#pragma once
#include <GLFW/glfw3.h>
#include <glm/vec2.hpp>
#include <optional>
#include <string>

std::optional<const std::string> read_file(const std::string& path);

struct Image {
    int width;
    int height;
    int channels;
    unsigned char* bytes;
};

std::optional<Image> read_image(const std::string& image);
