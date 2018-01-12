#include <iostream>

using namespace std;

template <int N>
void Print(const ARdouble (&arra)[N])
{
    for (const auto e : arra) {
         cout<<std::to_string(e)<<"  ";
    }
    cout<<endl;
}

template <int N,int M>
void PrintTransform(const float (&arra)[N][M])
{
    for(int i = 0 ;i < N ;i++){
        for(int j = 0 ; j < M ; j++){
            cout<<arra[i][j]<<"  ";
        }
    }
    cout<<endl;
}
