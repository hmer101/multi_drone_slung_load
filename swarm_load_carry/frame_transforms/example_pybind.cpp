/*
* To build this module, navigate to the build directory and run: cmake .. && make
*/

int addInner(int i, int j){
    return i + j;
}

int add(int i, int j) {
    return addInner(i, j);
}