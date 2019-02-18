#pragma once
#include <GLFW/glfw3.h>
#include <any>
#include <cstddef>
#include <functional>
#include <glm/vec2.hpp>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>

#define I(f)                                                                   \
    [&](auto&&... args) -> decltype(auto) {                                    \
        return f(std::forward<decltype(args)>(args)...);                       \
    }

namespace cli {
struct Option {
    Option(bool* flag): _flag(flag) {}
    virtual void parse(std::string s) = 0;
    virtual bool has_arg() = 0;
    void flag();
    bool see();

    bool* _flag;
    bool _seen = false;
};

template <typename T, typename F> struct OptionT: public Option {
    OptionT<T, F>(bool* flag, T* arg, F read):
        Option(flag),
        _arg(arg),
        _read(read) {}
    bool has_arg() { return _arg; }
    void parse(std::string s) {
        if (_read) {
            *_arg = _read(s);
        }
    }
    T* _arg;
    std::function<T(std::string)> _read;
};

template <typename T, typename F> Option* opt(bool* flag, T* arg, F read) {
    return new OptionT<T, F>(flag, arg, read);
}
template <typename F> Option* opt(bool* flag, std::nullptr_t, F read) {
    return opt<int>(flag, (int*)nullptr, read);
}

typedef std::unordered_map<std::string, Option*> Options;
}; // namespace cli

std::optional<const std::string> read_file(const std::string& path);

struct Image {
    int width;
    int height;
    int channels;
    unsigned char* bytes;
};

std::optional<Image> read_image(const std::string& image);
