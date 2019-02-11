# Turncoat Style Guide (C++)

---

## Principles

1. Be consistent more than anything else.
1. Code is read/diffed more often than it is written.
1. Write code for a future you (with amnesia, dyslexia, and malice.)
   > "You know you're brilliant but maybe you'd like to understand what you did
   > 2 weeks from now."
   > Linus Torvalds
1. Explicit over Implicit.
1. Flat over Nested

   So, present important information on the **left**.

1. Straightforward over Convoluted.

   So, prefer vertically compacted information as is reasonable.

1. Errors AND warnings (from gcc/clang/engine) should be loud.
1. Typography implies Semantics.
1. Prefer being language-idiomatic unless it hampers requirements.
1. You can effectively follow the Rust variant of PEP-8 except applying it to
   C++ with LLVM's high-level heuristics; however, influences come from Prettier,
   AirBnB JS, Black, Lua, Markdown, and Mozilla:

   - [Rust](https://github.com/rust-dev-tools/fmt-rfcs/blob/master/guide/guide.md)
   - [Pythonic PEP-8](https://www.python.org/dev/peps/pep-0008/)
   - [LLVM](https://llvm.org/docs/CodingStandards.html)
   - [Prettier](https://prettier.io/)
   - [AirBnB JS](https://github.com/airbnb/javascript)
   - [Black](https://black.readthedocs.io/en/stable/)
   - [Lua](http://lua-users.org/wiki/LuaStyleGuide)
   - [Mozilla](https://developer.mozilla.org/en-US/docs/Mozilla/Developer_guide/Coding_Style)

   Note, my style is meant to be cross-linguistically useful, so I remove any
   project specific things. LLVM, however, has many style points that I
   virtually copy to here.

   Changes to `.clang-format`:

   ```diff
   + BasedOnStyle: LLVM
   ```

1. These are only guidelines, so explicitly ignore them if it helps.

---

## Important

### Avoid Righthand Alignments

Anything remotely important enough to align should be on the left of my screen
where my eyeballs are reading vertically. Thus, important things go to the left,
and unimportant things go to the right. This leads to a complete and total
aversion from "visual" indenation, so I prefer "block" indentation as
demonstrated in rustfmt's defaults.

When my eyes are scanning they are reading vertically, not jumping sideways.
Within code, unlike text, horizontal jumps tend to be far more drastic, so I
prefer information to be compact and vertically oriented. I prefer this because
it lets me read more code as I'm scanning over it. This eases the need for page
jumps that require recalling off-screen regions' purpose.

**GOOD:**

```c++
functionator(
    abcdefg,
    hijkllmnop,
    qrst,
    uvwxyz
);
```

_BAD:_

```c++
functionator(abcdefg,
             hijklmnop,
             qrst,
             uvwxyz);
```

Changes to `.clang-format`:

```diff
+AlignAfterOpenBracket: AlwaysBreak
+AlignOperands: false
+AlignTrailingComments: false
+AlwaysBreakBeforeMultilineStrings: true
+BreakConstructorInitializers: AfterColon # No visual indentation!
+PointerAlignment: Left
+PenaltyReturnTypeOnItsOwnLine: 1000
```

### Semantic Lists should look like Typographic Lists

Because of Principle 8, writing out a list as a horizontal or vertical list will
imply its meaning.

**GOOD:**

```c++
Horizontal(abcdefg, hijklmn, opqrst, uvwxyz);
Vertical(
    abcdefg,
    hijklmn,
    opqrst,
    uvwxyz
);
```

_BAD:_

```c++
Textual(abcdefg, hijklmn,
    opqrst, uvwxyz);
```

If this "write it as a list" style bothers you, I encourage this heuristic:

> Try creating variables to capture complex arguments with a descriptive name

Thus, you **should** use this:

```c++
int complex_var = complex_expression == complex_expression;
fun(complex_var, var, i);
```

_instead of_ this:

```c++
fun(
    complex_expression == complex_expression,
    var,
    i
);
```

For this item, you must end initializer lists with a trailing comma, but only
if it doesn't fit on one line.

Changes to `.clang-format`:

```diff
+AllowShortCaseLabelsOnASingleLine: true
+BreakBeforeBinaryOperators: NonAssignment # Not All to be like structs/funcs
+ConstructorInitializersAllOnOneLineOrOnePerLine: true
+DanglingParenthesis: true # only with the 8.0.0 fork
```

### Semantic Blocks should look like typographic block-indents

**GOOD:**

```c++
//                   max line
int funct(              //
    parameters,         //
    options,            //
    arguments,          //
    error_codes         //
) {                     //
    code;               //
    return 0;           //
}                       //
                        //
struct Treasure {       //
    int swords;         //
    int potions;        //
};                      //
                        //
funct(                  //
    params,             //
    temp_options,       //
    args,               //
    errs                //
);                      //
code;                   //
code;                   //
```

An ignorable disadvantage to this style is that vertical space is used faster.
_Compare:_

```c++
int funct(parameters, options,   //
          arguments,             //
          error_codes) {         //
    code;                        //
    return 0;                    //
}                                //
                                 //
struct Treasure { int swords;    //
                  int potions; };//
                                 //
funct(params, temp_options,      //
      args, errs);               //
code;                            //
code;                            //
```

Note the accepted style, though, is how (complex) structs are usually written.
Because both a parameter list and a type specification are similar semanically,
Principle 8 claims they should be similar typographically. Arguably, both
specify a "package' of data or some kind of data layout:

- Structs stuff variables into a space in memory.
- Functions stuff arguments into their stack when invoked.

This sort of listing also includes continued control flow expressions, although
sadly clang-format does not neatly support this without my fork of it. You can
look at Prettier (and to a similar extent, `rustfmt`) for examples of this. As a
heuristic, treat the binary operators as the comma in a function definition,
except place them at the start of lines (because knowing which operator is used
is important).

**GOOD:**

```c++
while ( // blank comments needed for non-forked clang-format
    foobarbaz > 0
    && rumpus_fumpus_doodle_doo->foo(barballer) //
    && x
) {
    code();
    code();
}
```

_BAD:_

```c++
while (foobarbaz > 0
       && rumpus_fumpus_doodle_doo->foo(barballer) && x) {
    code();
    code();
}
```

Furthermore, this includes continued dot-chains or lessless-chains. Again,
clang-format does not natively support this in 8.0.0, so it's best to look at
Prettier or `rustfmt` for this.

**GOOD:**

```c++
foobarbaz  // blank comment needed without fork of 8.0.0
    .foobar()
    .foobar()
    .foobar()
    .foobar()
    .foobar();

foobarbaz //
    << MAJOR_VERSION_NUMBER << "."
    << MINOR_VERSION_NUMBER << "\n";
```

_BAD:_

```c++
foobarbaz.foobar()
    .foobar()
    .foobar()
    .foobar()
    .foobar();

foobarbaz << MAJOR_VERSION_NUMBER << "."
          << MINOR_VERSION_NUMBER << "\n";
```

Changes to `.clang-format`:

```diff
+AllowAllParametersOfDeclarationOnNextLine: false
+# AllowAllArgumentsOfCallOnNextLine: false # only with future 8.0.0 fork
+BinPackArguments: false
+BinPackParameters: false
+# BinPackOperators: false # only with future 8.0.0 fork
+# ChainAlignment: AlwaysBreak # only with future 8.0.0 fork
```

Note that the `AllowAllParametersOfDeclarationOnNextLine` option should also be
for function calls, as `rustfmt` would do.

#### Format lambdas like blocks of code

```c++
needs_callback(42, args, [&]() {
    code;
    return 4;
});
```

### Alignment of Syntax

The semantics of these expressions are implied from the appropriate typography,
as encouraged by Principle 8.

Left-align colons followed by a list as this is the grammatical English way of
using a colon as an enumerator or label. For colons which act more as a
partition or ratio, use spaces on both sides, except double colons as those
chain or concatenate together two things.

Left-align pointer and reference types, as it is more appropriate to think of
the identifier as a pointer to something, rather than the dereferenced
identifier being the something.

**GOOD:**

```c++
class Foo: public Bar {
public:
    Foo(int* i, std::vector<int> v): _x(*i > 0 ? -1 : 1), _v(v) {
        for (auto& j : _v) {
            j += _x;
        }
    };
private:
    int _x;
    std::vector<int> _v;
};
```

_BAD:_

```c++
class Foo : public Bar {
public :
    Foo(int *i, std :: vector<int> v) : _x(*i > 0 ? -1:1) {
        for (auto& j: v) {
            j += _x;
        }
        _v = v;
    };
private :
    int _x;
    std :: vector<int> _v;
};

```

Changes to `.clang-format`:

```diff
+PointerAlignment: Left
+SpaceBeforeCtorInitializerColon: false # only with 8.0.0
+SpaceBeforeInheritanceColon: false # only with 8.0.0
```

### One Line implies roughly one logical unit

Again, because of Principle 8. Thus, closing braces are helpful to see because
they denote lexical scoping, cleanup, and destruction. Packing arguments or
parameters only makes sense if items on the same line are semantically related,
at which point they should just be a structured type. So, either in the sense of
"these are all parameters" or "these are X parameters".

### Comment profusely! Explain why, not how!

Always use single-line ones (this makes debugging easier, and it's easier to see
what's commented).

It's very helpful to explain the purpose of a class or function.

### Add copyright headers

Like this:

```c++
// Copyright (c) 20yy-20yy Dalton Hildreth
// This file is under the MIT license. See the LICENSE file for details.
```

### Profusely use the latest (stable) tech in languages

Even C++17 is occassionally used throughout this repo. Lately C++ has improved
its expressiveness, which just makes programming more pleasant and simpler.

For example, use const(expr) correctness and ownership semantics. Generally use
`unique_ptr` with weak `*` references to them. Occassionally use `shared_ptr`
with `weak_ptr` references to them.

### Prefer composition over (implementation) inheritance

Inheritance should be very carefully applied, and likely not for code re-use.

---

## High-Level

### Header Include Order

Generally go fro specific to generic: local > module > project > system. For
example:

```c++
// local
#include "impl.h"
// module
#include "help.h"
#include "lower/helper.h"
// project
#include "../module/thing.h"
// third party
#include <thirdparty/shenanigans>
#include <glm/glm.hpp>
// system
#include <memory>
// c system
#include <cstdlib>
// OS - specific
#include <windows.h>
```

Changes to `.clang-format`:

```diff
+IncludeCategories:
+  - Regex:            '^".*"'
+    Priority:         1
+  - Regex:            '^<.*\.h"'
+    Priority:         2
+  - Regex:            '<.*'
+    Priority:         3
+  - Regex:            '.*'
+    Priority:         4
```

### Headers should be self-contained; use .inc for ones that are not

### (Often) no RTTI, (Often) no exceptions

They confuse logic, twisting it up like noodles and bogging down the program.

As far as error handling goes, bitsquid dev Niklas gives good advice:

- [Part 1: Intro & Unexpected Errors](http://bitsquid.blogspot.com/2012/01/sensible-error-handling-part-1.html)
- [Part 2: Expected Errors](http://bitsquid.blogspot.com/2012/02/sensible-error-handling-part-2.html)
- [Part 3: Warnings](http://bitsquid.blogspot.com/2012/02/sensible-error-handling-part-3.html)

### Write cross-platform, portable code. Isolate it where you cannot

### DO NOT use static constructors (globals with them)

This needlessly impacts performance and memory use.

### Limit the use of `struct` to POD-like types

### `#include` as little as possible, but don't bother with forward decls

### Keep "internal" headers/interfaces private

Don't put them in include/

### Exit early and continue to simplify code

The same goes for not using an else right after an early-jump. This is largely
because of Principle 7.

### Fail Early

Assert, unit test, and play test. A Lot.

You should use Catch2, and even if I'm not good at it doesn't mean you should be
too! Admittedly, I will set this up better sooner than later.

---

## Low-Level

### Naming should be descriptive without being superfluous

Case definition:

- Normal: HTML, C-Space, Two Words
- snake_case: html, cspace, two_words
- \_leading_case: \_html, \_cspace, \_two_words
- SCREAM_CASE: HTML, CSPACE, TWO_WORDS
- CamelCase: Html, Cspace, TwoWords
- trailing_case\_: html\_, cspace\_, two_words\_
- nonecase: html, cspace, twowords

Generally, values = snake_case and Types = CamelCase

- Types: CamelCase
- Namespaces: nonecase
- Functions:
  - Methods: snake_case
  - Private Methods: \_leading_case
  - Namespaced: snake_case
  - Globals: SCREAM_CASE
  - File Static: \_leading_case
  - Getter (rare): Type var_name()
  - Setter (rare): void var_name(Type v)
- Variables:
  - Private Members: \_leading_case
  - Locals: snake_case
  - Globals: SCREAM_CASE
  - Iters: i j k
  - Vectors: (w) x y z, u v w
  - Color: r g b a, h s v
  - Temporary: t s
  - Random: r
  - File: f

To clarify word boundaries, if it was originally hyphenated, use one word.
e.g. Configuration-space -> C-Space -> \_cspace
If it's an acronym, treat like one word.
Hyper Text Markup Language -> HTML -> \_html
Probabilistic Roadmap -> PRM -> Prm

### Use `#pragma once` instead of header guards

### Use range-based for loops

```c++
for (const auto& val : container) {
    observe(val);
}
for (auto& val : container) {
    val.change();
}
for (auto copy : container) {
    observe(/*small*/copy);
    copy.change();
    save(copy);
}
for (auto&& val : container) {
    proxy_iter_or_generic.change();
}
```

### Use `auto` if it helps readability.

Especially helpful for redudant lines or generics

### Use predicate functions instead of predicate loops

The LLVM style guide [gives good reasoning](https://llvm.org/docs/CodingStandards.html#turn-predicate-loops-into-predicate-functions)
for this.

**GOOD:**

```c++
std::array<int, 5> a = {5, 4, 3, 1, 2};
if (!std::is_sorted(a.cbegin(), a.cend(), [](int a, int b){ return a > b; })) {
    printf("a is unsorted\n");
}
```

It's even better to write a function for predicates you often use. I may also
write a wrapper library for `<algorithm>` whose API looks closer to this:
`std::is_sorted(iterable, /* ComparePred */);`

_BAD:_

```c++
std::array<int, 5> a = {5, 4, 3, 1, 2};
for (unsigned i = 0; i < a.size() - 1; ++i) {
    if (a[i] <= a[i + 1]) {
        printf("a is unsorted\n");
        break;
    }
}
```

### _DO NOT_ use `using` in a header at an outer namespace/global scope

However, using any "using" is typically avoided except for exceptional
readibility issues. (This is an awkward trade-off between P2. and P1.)

### _NEVER_ `#include <iostream>` either use `fmt` or `cstdio`

iostream is slow, excessive, and has no clear benefits

### Don't add inline to functions in the class definition

The `inline` is implicit for member functions define in a class definition.

**GOOD:**

```c++
class Foo {
public:
    void bar() { do_stuff(); };
};
```

_BAD:_

```c++
class Foo {
public:
    inline void bar() { do_stuff(); };
};
```

### Prefer using enums when passing literal arguments instead of bools

At least, if it is not obvious at the call-site

### Use explicit on a single argument constructor that isn't a type-cast

### Use constructor initializers

They are performant, and they don't hurt readability

---

## Trivial

### Line Length

Max 80 char lines. I like to split my screen a lot. Helps with P4., too. I'm
fine with specific projects or regions using 100 char lines, though.

### Indentation

4-SPACEs indent, because it is like PEP-8, Rust, and the mode of C++ users
according to this source:

- [Tabs or Spaces](https://ukupat.github.io/tabs-or-spaces/)

Though I have always used 4-spaces, I may switch to 2-space in specific projects
because it is what these use:

- LLVM
- HTML/CSS/JS
- Lua
- Bash
- Mozilla
- Markdown
- Wren

Changes to `.clang-format`:

```diff
+AccessModifierOffset: -4
+ConstructorInitializerIndentWidth: 4
+ContinuationIndentWidth: 4 # Awkward without DanglingParentheses: true
+IndentWidth: 4
+IndentPPDirectives: AfterHash
```

### No trailing whitespace

Neither at the end of lines or on blank lines.

### Spaces before parentheses

Only do this before control-flow expressions, NOT functions.

**GOOD:**

```c++
if (foo < bar) {
    baz();
}
```

_BAD:_

```c++
if(foo < bar){
    baz ();
}
```

### Use preincrement, unless you NEED postincrement

### Anonymous namespaces

They should be small. Prefer static for random functions.

### Fully write out literals (such as .f for floats)

It is better to be explicit about literal types, as per Principle 4. This is
also because adding the literals can change the type sometimes.

### Be lazy with type modifiers where it's obvious

`unsigned` over `unsigned int`, as per Princple 6. There is no point in writing
int as `unsigned` unambiguously denotes only one type.

### Use nullptr in C++; check for it without comparators

**GOOD:**

```c++
if (!a) {
    a->do_things();
}
```

_BAD:_

```c++
if (a != nullptr) {
    a->do_things();
}
```
