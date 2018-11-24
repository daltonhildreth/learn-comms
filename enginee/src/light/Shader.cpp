#include "Shader.h"

#include <iostream>
#include <glm/gtc/type_ptr.hpp>
#include "../io.h"

#include "../util/debug.h"

using namespace std;

Shader::Shader()
    : program(0)
    , success(false) {
}

Shader::~Shader() {
    for (auto& sobj : shader_objects) {
        glDetachShader(program, sobj.first);
        glDeleteShader(sobj.first);
        sobj.first = 0;
    }
    shader_objects.clear();
    glDeleteProgram(program);
    program = 0;
    success = false;
}

bool Shader::add(GLenum type, const std::string path) {
    if (file_types.count(path)) {
        cerr << "gg! Shader already being used by this shader program.\n";
        return false;
    }
    file_types[path] = type;
    return true;
}

bool Shader::del(const std::string path) {
    if (!file_types.erase(path)) {
        cerr << "gg! Shader " << path << " not in program to delete.\n";
        return false;
    }
    return true;
}

bool Shader::build() {
    program = glCreateProgram();
    shader_objects.clear();
    for (const auto& pair : file_types) {
        compile(pair.second, pair.first);
    }
    if (success) {
        link();
    }
    return success;
}

void Shader::use() {
    assert(program != 0 && "gg! Program not built\n");
    glUseProgram(program);
}

bool Shader::compile(GLenum type, const string& path) {
    string content = *read_file(path);

    GLuint shader_object = glCreateShader(type);
    const char* c = content.c_str();
    glShaderSource(shader_object, 1, &c, nullptr);
    glCompileShader(shader_object);

    //check for errors
    glGetShaderiv(shader_object, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(shader_object, 512, nullptr, info_log);
        cerr << "gg! Shader (" << path << ") failed to compile:\n"
             << info_log << "\n";
    }
    shader_objects.push_back(make_pair(shader_object, path));
    return success;
}

bool Shader::link() {
    for (auto& sobj : shader_objects) {
        glAttachShader(program, sobj.first);
    }
    glLinkProgram(program);

    //check for errors
    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (!success) {
        glGetProgramInfoLog(program, 512, nullptr, info_log);
        cerr << "gg! Shaders failed to link:\n" << info_log << "\n";
    }
    return success;
}

std::optional<GLint> Shader::check_uniform(string uniform) {
    if (uniform.substr(3) == "gl_") {
        cerr << "gg! Shaders cannot use gl_ before uniforms " << uniform << "\n";
        return std::nullopt;
    }
    GLint loc = glGetUniformLocation(program, uniform.c_str());
    if (loc == -1) {
        cerr << "gg! Shaders do not contain uniform " << uniform << "\n";
        return std::nullopt;
    }
    return loc;
}

void Shader::set(string uniform, GLint i) {
    glUniform1i(*check_uniform(uniform), i);
}

void Shader::set(string uniform, GLuint u) {
    glUniform1ui(*check_uniform(uniform), u);
}

void Shader::set(string uniform, GLfloat f) {
    glUniform1f(*check_uniform(uniform), f);
}

void Shader::set(string uniform, glm::vec3 v) {
    glUniform3f(*check_uniform(uniform), v.x, v.y, v.z);
}

void Shader::set(string uniform, glm::mat4 m) {
    //supplied in column major order
    glUniformMatrix4fv(*check_uniform(uniform), 1, GL_FALSE, glm::value_ptr(m));
}
