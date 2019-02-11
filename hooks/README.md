# Git Client Hooks for Turncoat

Client-side git hooks to help automate trivial work on Turncoat.

## Install

```sh
chmod +x hooks/pre-commit
chmod +x hooks/pre-commit-clang-format.sh
cd .git/hooks
rm pre-commit.sample
ln -s ../../hooks/pre-commit
ln -s ../../hooks/pre-commit-clang-format.sh
```
