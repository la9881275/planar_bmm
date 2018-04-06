reset;
rm -rf build/;
mkdir build && cd $_;
cmake ..;
make -j8;
cd bin/