Good Game Engine / 'The Flashy Thing' / Winter...? Style Guide (for C++)
- Dalton Hildreth

---

Whitespace
use
    (4 space indentation)
not
	(1 tab indentation)

---

Header order
~~~~
//local
#include "declare.h"
//your module
#include "otherthing.h"
#include "lower/thing.h"
//project
#include "../module/ThingINeed.h"
//3rd pary
#include <thirdparty/shenanigans>
#include <glm/glm.hpp>
#include <GLFW/glfw3.h>
//system
#include <memory>
#include <vector>
//c system
#include <cstdlib>
//OS - probably shouldn't need
//#include <windows.h>

//Forward declare.
class Forward;
class Soldiers;
class March;
~~~~

---

Naming

- Types: `CamelCase`
In general: `snake_case`, `_under_snake_case` if encapsulated.
- Functions:
    - Methods: `snake_case`
    - Getter: `Type var_name()`
    - Setter: `void var_name(Type v)`
    - Globals: `snake_case`
    - Static: `_under_snake_case`
- Variables: 
    - Members: `_under_snake_case`
    - Locals: `snake_case`
    - Parameters: `snake_case`
    - Globals: `CAPITAL_CASE`
    To clarify word boundaries, if it was originally hypenated, use one word.
    e.g. Configuration-space -> C-Space -> _cspace
    If it's an acronym, treat like one word.
    Hyper Text Markup Language -> HTML -> _html
    Probabilistic Roadmap -> PRM -> Prm 

---

Testing

You should unit test with Catch. Just because I'm not good at it doesn't mean
you should be too! Admittedly I will set this up better sooner than later.

---

Bracing 

K&R -

~~~~
if (a) {
    return b;
} else {
    return c;
}
~~~~

Line continuation

~~~~
if (a && abcdefghijklmnopqrstuvwxyz
        && x && z) {
    return "still got it!";
}

//declare
void f(int a, int b, int c, int d,
    int f);
//define
void f(int a, int b, int c, int d,
        int f) {
    hey;
}
~~~~

---

Safety/Types

try to use smart pointers and ownership semantics.
Generally use `unique_ptr`, with weak `*` references to them
or occasionally `shared_ptr`, with `with_ptr` references to them.

---
