#include <iostream>
#include <math/transform.h>

int main() {
    std::cout << "Hello, World!" << std::endl;
    rzpbr::Matrix a(0.0);
    for(auto i = 0;i < 4;i++){
        for(auto j = 0;j < 4;j++){
            if(i == j){
                a.set(i,j,3);
            } else if (j == 3){
                a.set(i, j, 4);
            }
        }
    }

    std::cout<<a<<std::endl;

    auto ai = rzpbr::inverse(a);

    std::cout<<ai<<std::endl;

    auto re = ai * a;

    std::cout<<re<<std::endl;

    return 0;
}
