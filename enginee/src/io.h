#pragma once
#include <GLFW/glfw3.h>
#include <algorithm>
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
    Option(bool* flag, std::string arg_name):
        _flag(flag),
        _arg_name(arg_name) {}
    virtual void parse(std::string s) = 0;
    virtual bool has_arg() = 0;
    void flag();
    bool see();

    bool* _flag;
    bool _seen = false;
    std::string _arg_name = "";
};

template <typename T, typename F> struct OptionT: public Option {
    OptionT<T, F>(bool* flag, T* arg, F read, std::string arg_name):
        Option(flag, arg_name),
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

struct Positional {
    Positional(std::string arg_name): _arg_name(arg_name) {}
    virtual void parse(std::string s) = 0;
    bool see();
    bool _seen = false;
    std::string _arg_name = "";
};

template <typename T, typename F> struct PositionalT: public Positional {
    PositionalT<T, F>(T& arg, F& read, std::string arg_name):
        Positional(arg_name),
        _arg(arg),
        _read(read) {}
    void parse(std::string s) { _arg = _read(s); }
    T& _arg;
    std::function<T(std::string)> _read;
};

template <typename T, typename F>
Option* opt(bool* flag, T* arg, F read, std::string arg_name) {
    return new OptionT<T, F>(flag, arg, read, arg_name);
}
Option* opt(
    bool* flag,
    std::nullptr_t = nullptr,
    std::nullptr_t = nullptr,
    std::string = ""
);

template <typename T, typename F>
Positional* pos(T& arg, F read, std::string arg_name) {
    return new PositionalT<T, F>(arg, read, arg_name);
}

typedef std::unordered_map<std::string, Option*> Options;
typedef std::vector<Positional*> Positionals;

void parse(int argc, char** argv, Options&& opts, Positionals&& poss);
void parse(int argc, char** argv, Options& opts, Positionals& poss);
}; // namespace cli

std::optional<const std::string> read_file(const std::string& path);

struct Image {
    int width;
    int height;
    int channels;
    unsigned char* bytes;
};

std::optional<Image> read_image(const std::string& image);
