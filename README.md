# viterbi-snakes
Simple implementation of active contours (snakes) based on dynamic programming

## How to build

```
#install conan, needed only once
pip install conan

#from the repo root
conan install -if build --build missing .
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=install -G "Ninja" ..
ninja install 

```

## Reference paper

Amini, A. & Weymouth, Terry & Jain, Ramesh. (1990). Using Dynamic Programming for Solving Variational Problems in Vision. Pattern Analysis and Machine Intelligence, IEEE Transactions on. 12. 855-867. 10.1109/34.57681. 

