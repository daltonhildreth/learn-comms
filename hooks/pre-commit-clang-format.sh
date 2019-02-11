#!/bin/sh

# git pre-commit hook that runs a clang-format stylecheck.
# Features:
#  - abort commit when commit does not comply with the style guidelines
#  - create a patch of the proposed style changes
#
# More info on Clang-format: https://clang.llvm.org/docs/ClangFormat.html
# changes for clang-format by Dalton Hildreth, djdjhildreth@gmail.com

# This file is part of a set of unofficial pre-commit hooks available
# at github.
# Link:    https://github.com/ddddavidmartin/Pre-commit-hooks
# Contact: David Martin, ddddavidmartin@fastmail.com

# CONFIGURATION
# Set path to clang-format binary
CLANG_FORMAT="./tools/build/bin/clang-format"

# Remove any older patches from previous commits. Set to true or false.
DELETE_OLD_PATCHES=false

# Only parse files with the extensions in FILE_EXTS. Set to true or false.
# If false every changed file in the commit will be parsed with clang-format.
# If true only files matching one of the extensions are parsed with clang-format.
PARSE_EXTS=true

# File types to parse. Only effective when PARSE_EXTS is true.
FILE_EXTS=".c .h .cpp .hpp .inc"

# exit on error
set -e

# check whether the given file matches any of the set extensions
matches_extension() {
    local filename=$(basename "$1")
    local extension=".${filename##*.}"
    local ext

    for ext in $FILE_EXTS; do [ "$ext" = "$extension" ] && return 0; done

    return 1
}

# necessary check for initial commit
if git rev-parse --verify HEAD >/dev/null 2>&1 ; then
    against=HEAD
else
    # Initial commit: diff against an empty tree object
    against=4b825dc642cb6eb9a060e54bf8d69288fbee4904
fi

if ! command -v "$CLANG_FORMAT" > /dev/null ; then
    printf "Error: clang-format executable not found.\n"
    printf "Set the correct path in "$0".\n"
    exit 1
fi

# create a random filename to store our generated patch
prefix="pre-commit-clang-format"
suffix="$(date +%s)"
patch="/tmp/$prefix-$suffix.patch"

# clean up any older clang-format patches
$DELETE_OLD_PATCHES && rm -f /tmp/$prefix*.patch

# create one patch containing all changes to the files
git diff-index --cached --diff-filter=ACMR --name-only $against -- | \
while read file; do
    # ignore libraries
    if cat $file | grep -q "$(dirname "$0")/lib"; then
        continue;
    fi

    # ignore file if we do check for file extensions and the file
    # does not match any of the extensions specified in $FILE_EXTS
    if $PARSE_EXTS && ! matches_extension "$file"; then
        continue;
    fi

    # clang-format our sourcefile, create a patch with diff and append it to our
    # $patch
    # The sed call is necessary to transform the patch from
    #    --- $file timestamp
    #    +++ - timestamp
    # to both lines working on the same file and having a/ and b/ prefix.
    # Else it can not be applied with 'git apply'.
    "$CLANG_FORMAT" -style=file "$file" | \
        diff -u "$file" - | \
        sed -e "1s|--- |--- a/|" -e "2s|+++ -|+++ b/$file|" >> "$patch"
done

# if no patch has been generated all is ok, clean up the file stub and exit
if [ ! -s "$patch" ] ; then
    printf "Files in this commit comply with the clang-format rules.\n"
    rm -f "$patch"
    exit 0
fi

# a patch has been created, notify the user and exit
printf "\nThe following differences were found between the code to commit "
printf "and the clang-format rules:\n\n"
cat "$patch"
printf "\n"

# Allows us to read user input below, assigns stdin to keyboard
exec < /dev/tty

while true; do
    echo "Do you want to apply that patch?"
    echo "[a]: Apply the patch"
    echo "[f]: Force and commit anyway (not-recommended)"
    echo "[c]: Cancel commit"
    read -r answer < /dev/tty

    case "$answer" in
    [aA] )
        git apply $patch;
        printf "The patch was applied. You can now stage the changes and ";
        printf "commit again.\n\n";
        break
        ;;
    [fF] )
        echo "Abort the force by leaving a blank message on the commit."
        echo "Press enter to continue..."
        read -r < "$tty"
        exit 0
        ;;
    [cc] )
        printf "\nYou can apply these changes with:\n";
        printf "  git apply $patch\n";
        printf "(may need to be called from the root directory of the repo)\n";
        printf "Aborting commit. Apply changes and commit again or skip this ";
        printf "check with --no-verify (not recommended).\n\n";
        break
        ;;
    * ) echo "Please answer yes or no."
        ;;
    esac
done
exit 1