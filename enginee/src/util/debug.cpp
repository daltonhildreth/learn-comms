#ifdef DEBUG
#include "debug.h"
#include <glad.h>
#include <iostream>
using namespace std;

void glcheck(const char* loc) {
    switch (glGetError()) {
    case GL_INVALID_ENUM:
        cerr << "Invalid Enum " << loc << "\n";
        break;
    case GL_INVALID_VALUE:
        cerr << "Invalid value " << loc << "\n";
        break;
    case GL_INVALID_OPERATION:
        cerr << "Invalid operation " << loc << "\n";
        break;
    case GL_STACK_OVERFLOW:
        cerr << "Invalid stack overflow " << loc << "\n";
        break;
    case GL_STACK_UNDERFLOW:
        cerr << "Stack underflow " << loc << "\n";
        break;
    case GL_OUT_OF_MEMORY:
        cerr << "Out of memory " << loc << "\n";
        break;
    case GL_TABLE_TOO_LARGE:
        cerr << "Table too large " << loc << "\n";
        break;
    default:
        clog << "No error" << loc << "\n";
        break;
    }
}   
#endif
