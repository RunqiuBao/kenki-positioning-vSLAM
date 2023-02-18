echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j6

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j6

cd ../../../

echo "Configuring and building ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j6

cd ..

#echo "Converting vocabulary to binary"
#./tools/bin_vocabulary
