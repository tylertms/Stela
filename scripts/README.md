### Arch Linux Issues
Arch Linux has some issues with building. So this folder, along with the relevant arch.sh script, helps fix those building issues on Arch Linux. 
The main README says to use Clang23+. Issue is, Clang23+ doesn't really work for building on Arch Linux.
So instead, we need to use Clang21, LLVM21, and LLD21. 
That apparently fixes it and let's the build continue/go through.


# How to use
Clone Stela like usual. Then just source scripts/arch.sh or wherever the arch.sh file is.


# Building
```cargo xtask dist 2>&1 | tee ~/stela-build.log```
