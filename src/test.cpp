#include <iostream>
#include <fstream>
#include <bitset>


unsigned int getbits(unsigned int x, int p, int n) {
    unsigned int res = (x << p) & (~0 << (32-n));
    std::cout << std::bitset<32>{res} << '\n';
    res = (res >> (32-n));
    std::cout << std::bitset<32>{res} << '\n';
    return res;
}

void convert(unsigned int* a) {
    *a = (*a << 24) | ((*a << 8) & 0xFF0000) | ((*a >> 8) & 0xFF00) | (*a >> 24);
}

int main() {

    unsigned char buffer[10];
    std::ifstream data("../data/COM3_210730_115228.ubx", std::ios::binary);

    data.read(reinterpret_cast<char *>(&buffer), sizeof(buffer));

    for (int i=0; i<10; i++){
        std::cout << (int)buffer[i] << "\t" << std::bitset<8>{buffer[i]} <<std::endl;
    }

    std::cout << "\n\n\n";

    data.seekg(-4, std::ios::cur);

    data.read(reinterpret_cast<char *>(&buffer), sizeof(buffer));

    for (int i=0; i<10; i++){
        std::cout << (int)buffer[i] << "\t" << std::bitset<8>{buffer[i]} <<std::endl;
    }




    /*

    unsigned int *deneme = reinterpret_cast<unsigned int *>(&buffer);
    convert(deneme);

    std::cout << std::bitset<32>{*deneme} << '\n';

    unsigned int b = getbits(*deneme, 2, 6);

    std::cout << std::bitset<8>{b} << '\n';

    std::cout << b << std::endl;

    std::cout << "Size: " << sizeof(*deneme) << std::endl;

    */


    return 0;
}