#include "io.h"
#include <cstdio>
#include <cstdlib>
#include <experimental/optional>
#include <fstream>
#include <iostream>
#include <numeric>
#include <stb_image.h>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>
#undef GLM_ENABLE_EXPERIMENTAL

using namespace std;

namespace cli {
void Option::flag() {
    if (_flag) {
        *_flag = true;
    }
}

bool Option::see() {
    if (_seen) {
        return false;
    } else {
        return _seen = true;
    }
}

bool Positional::see() {
    if (_seen) {
        return false;
    } else {
        return _seen = true;
    }
}

Option* opt(bool* flag, nullptr_t, nullptr_t, string) {
    return opt<int>(flag, static_cast<int*>(nullptr), nullptr, "");
}

static void _error(string message) {
    fprintf(stderr, "%s\n", message.c_str());
    exit(EXIT_FAILURE);
}

static string _usage(string prog, Options& opts, Positionals& poss) {
    std::string s("Usage: " + prog);
    for (const auto& opt : opts) {
        s += " [--" + opt.first;
        if (opt.second->_arg_name != "") {
            s += " " + opt.second->_arg_name;
        }
        s += "]";
    }
    for (const auto& pos : poss) {
        s += " " + pos->_arg_name;
    }
    return s;
}

void parse(int argc, char** argv, Options&& opts, Positionals&& poss) {
    Options _opts(std::move(opts));
    Positionals _poss(std::move(poss));
    parse(argc, argv, _opts, _poss);
    for (auto& i : _opts) {
        delete i.second;
    }
    for (auto& i : _poss) {
        delete i;
    }
}
void parse(int argc, char** argv, Options& opts, Positionals& poss) {
    std::string usage = _usage(argv[0], opts, poss);

    auto pos = poss.begin();
    for (int opti = 1; opti < argc; ++opti) {
        std::string arg(argv[opti]);
        bool is_opt = arg.substr(0, 2) == "--";

        // read options
        cli::Options::iterator key;
        if (is_opt && (key = opts.find(arg.substr(2))) != opts.end()) {
            cli::Option& opt = *key->second;
            if (!opt.see()) {
                _error("no double options\n" + usage);
            }
            opt.flag();
            if (opt.has_arg()) {
                if (opti + 1 >= argc) {
                    _error("no arg to read\n" + usage);
                }
                arg = argv[++opti];
                if (arg.find("-") == std::string::npos) {
                    opt.parse(arg);
                } else {
                    _error("bad arg\n" + usage);
                }
            }

            // --help hack
            if (!opt._flag && !opt.has_arg()) {
                printf("%s\n", usage.c_str());
                exit(EXIT_SUCCESS);
            }

        } else {
            // read positionals
            if (pos == poss.end()) {
                _error("no further positionals\n" + usage);
            }
            if (arg.find("--") == std::string::npos) {
                (*pos)->see();
                (*pos)->parse(arg);
                ++pos;
            } else {
                _error("unknown option\n" + usage);
            }
        }
    }

    // check for required positionals
    if (any_of(poss.begin(), poss.end(), [](auto p) { return !p->_seen; })) {
        _error("missing required positional\n" + usage);
    }
}
} // namespace cli

experimental::optional<const string> read_file(const string& path) {
    ifstream file(path, ios::in | ios::binary);
    if (!file) {
        cerr << "gg! Failed to open file " << path << "\n";
        return experimental::nullopt;
    }

    file.seekg(0, ios::end);
    string content(static_cast<size_t>(file.tellg()), '\0');
    file.seekg(0, ios::beg);
    file.read(&content[0], static_cast<long int>(content.size()));
    if (!file) {
        cerr << "gg! Failed to read entire file " << path << "\n";
        return experimental::nullopt;
    }

    return content;
}

experimental::optional<Image> read_image(const string& path) {
    Image i;
    i.bytes = stbi_load(path.c_str(), &i.width, &i.height, &i.channels, 0);
    if (i.bytes) {
        return i;
    } else {
        return experimental::nullopt;
    }
}
