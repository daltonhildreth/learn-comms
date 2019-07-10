# Purpose of subdirectory tools/

Contain custom engine or forked 3rd party tools that help with developing
Turncoat and the gameplay or content of the games it helps to develop.

## llvm/clang clang-format fork

customized clang-format that supports the [DanglingParenthesis](https://reviews.llvm.org/D33029)
option as well as the latest options as-of version 8.0.0 of clang. Some minor
extra changes to the DanglingParenthesis option were made for consistency with
the style guide (guided based on how rustfmt would do block indentation).

Other extra options are planned to be more consistent with the style guide,
`rustfmt`, and other influences such as Prettier:

- BinPackOperators: Treats continued binary operators similarly to commas as in
  BinPackArguments or BinPackParameters. It will also leave the head of
  control flow statements on their own line, similar to Prettier.
- ChainAlignment: Same options as AlignAfterOpenBracket, except it applies to
  how chained `.`, `->`, `>>`, and `<<` operators are aligned after an
  identifier.

### How-To Build

Build this custom clang-format by running ./build.sh within the tools directory.

It runs largely as per the official [getting started](https://clang.llvm.org/get_started.html)
page of clang with minor convenience changes.
