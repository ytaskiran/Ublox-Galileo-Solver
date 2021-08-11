#include <iostream>
#include <fstream>
#include <bitset>


unsigned int getbits(unsigned int x, int p, int n) {
    return (x >> p) & ~(~0 << n);
}

int main() {

    unsigned char buffer[4];
    std::ifstream data("../data/COM3_210730_115228.ubx", std::ios::binary);

    data.read(reinterpret_cast<char *>(&buffer), sizeof(buffer));

    for (int i=0; i<4; i++){
        std::cout << (int)buffer[i] << "\t" << std::bitset<8>{buffer[i]} <<std::endl;
    }

    unsigned int *deneme = reinterpret_cast<unsigned int *>(&buffer);
    std::cout << std::bitset<32>{*deneme} << '\n';

    unsigned int a = getbits(*deneme, 2, 6);

    std::cout << std::bitset<8>{a} << '\n';

    std::cout << a << std::endl;

    std::cout << "Size: " << sizeof(*deneme) << std::endl;


    return 0;
}