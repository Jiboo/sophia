# Run from cmake build directory, so that clang tooling may access compile_commands.json

clang-tidy -fix -checks=readability*,performance*,modernize*,clang-analyzer*,bugprone*,-readability-braces-around-statements -header-filter=.* -format-style="{BasedOnStyle: llvm, ColumnLimit: 120}"  ../src/* ../tst/* ../tst/gtests/* ../cli/*
clang-format -i -style="{BasedOnStyle: llvm, ColumnLimit: 120}" ../src/* ../inc/* ../tst/* ../tst/gtests/* ../cli/*
