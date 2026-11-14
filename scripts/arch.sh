sudo pacman -Syu --needed clang21 llvm21 lld21 rustup git
rustup default stable

mkdir -p ~/stela-clang
for tool in clang clang++; do
cat > ~/stela-clang/$tool <<EOF
#!/bin/sh
# Arch's clang enables -fstack-protector-strong by default; Stela's C library has no __stack_chk_fail.
for arg in "\$@"; do
  case "\$arg" in --version|-dumpversion|-dumpmachine|-print-*|--print-*|-###) exec /usr/lib/llvm21/bin/$tool "\$@" ;; esac
done
exec /usr/lib/llvm21/bin/$tool -fno-stack-protector "\$@"
EOF
chmod +x ~/stela-clang/$tool
done

cat > ~/stela-env.sh <<'EOF'
export PATH=$HOME/stela-clang:/usr/lib/llvm21/bin:$PATH
export CLANG=$HOME/stela-clang/clang
export LIBCLANG_PATH=/usr/lib/llvm21/lib
EOF

source ~/stela-env.sh
[ -d ~/Stela ] || git clone https://github.com/tylertms/Stela ~/Stela
cd ~/Stela && cargo xtask dist 2>&1 | tee ~/stela-build.log
